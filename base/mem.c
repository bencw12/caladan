/*
 * mem.c - memory management
 */

#include <asm/mman.h>
#include <linux/mman.h>
#include <linux/shm.h>

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <numaif.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>

#include <base/stddef.h>
#include <base/mem.h>
#include <base/log.h>
#include <base/limits.h>
#include <base/syscall.h>

#if !defined(MAP_HUGE_2MB) || !defined(MAP_HUGE_1GB)
#warning "Your system does not support specifying MAP_HUGETLB page sizes"
#endif

#if !defined(SHM_HUGE_2MB) || !defined(SHM_HUGE_1GB)
#warning "Your system does not support specifying SHM_HUGETLB page sizes"
#endif

bool cfg_transparent_hugepages_enabled;
/* map runtime memory MAP_SHARED so it survives address-space cloning */
bool cfg_shared_runtime_mem = true;

/*
 * Called whenever the runtime maps memory for itself. Junction overrides this
 * to record LibOS allocations, which are the ones that have to be visible from
 * every guest address space.
 */
__weak void on_runtime_map(const char *what, void *addr, size_t len, int prot,
			   int flags)
{
	(void)what; (void)addr; (void)len; (void)prot; (void)flags;
}

#ifndef MFD_ALLOW_SEALING
#define MFD_ALLOW_SEALING 0x0002U
#endif
#ifndef F_ADD_SEALS
#define F_ADD_SEALS 1033
#define F_SEAL_SEAL 0x0001
#define F_SEAL_SHRINK 0x0002
#define F_SEAL_GROW 0x0004
#endif

struct runtime_mem_region runtime_lgpage_region = { .fd = -1 };
struct runtime_mem_region runtime_stack_region = { .fd = -1 };

/**
 * runtime_mem_region_init - creates a region's memfd and reserves its slot
 * @r: the region
 * @name: for the memfd and for logs
 * @base: where the region lives; must be free, and stays so
 * @len: the whole index space, however much of it is ever used
 * @granule: the unit one fault repairs (a page, a stack)
 *
 * Runs at init, before any guest exists and before the seccomp filter, so
 * libc is fine here. Returns 0 if successful, otherwise -errno.
 */
int runtime_mem_region_init(struct runtime_mem_region *r, const char *name,
			    uintptr_t base, size_t len, size_t granule)
{
	void *p;
	int fd;

	/* Sealed: the kernel enforces the size at fault time, not mmap time,
	 * so a size that could change would not be a bound. */
	fd = memfd_create(name, MFD_CLOEXEC | MFD_ALLOW_SEALING);
	if (fd < 0)
		return -errno;
	if (ftruncate(fd, len) < 0 ||
	    fcntl(fd, F_ADD_SEALS, F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL) < 0) {
		close(fd);
		return -errno;
	}

	/* One PROT_NONE reservation over the whole index space, made before
	 * the first clone so it exists in every address space. It costs
	 * address space only, and it keeps anything else out of the region.
	 * There is no pool to overflow: the limit is the index space. */
	p = mmap((void *)base, len, PROT_NONE,
		 MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE |
		 MAP_FIXED_NOREPLACE, -1, 0);
	if (p != (void *)base) {
		log_err("%s: could not reserve 0x%lx-0x%lx", name, base,
			base + len);
		if (p != MAP_FAILED)
			munmap(p, len);
		close(fd);
		return -ENOMEM;
	}

	r->name = name;
	r->base = base;
	r->len = len;
	r->granule = granule;
	r->fd = fd;
	log_info("%s: 0x%lx-0x%lx (%ld GB of address space) backed by fd %d",
		 name, base, base + len, len >> 30, fd);
	return 0;
}

/**
 * runtime_mem_region_map - maps a slice of a region from its memfd
 * @r: the region
 * @addr: the slice's fixed address, inside the region
 * @len: bytes
 *
 * The binding is (fd, addr - base), the same every time this address is
 * used, so mapping a slice that is already mapped -- here or anywhere else --
 * installs the identical mapping. That is what makes it safe for an address
 * space that still holds the old mapping: nothing is ever rebound.
 *
 * Returns @addr, or NULL on failure.
 */
void *runtime_mem_region_map(struct runtime_mem_region *r, void *addr,
			     size_t len)
{
	void *p;

	assert(r->fd >= 0);
	assert((uintptr_t)addr >= r->base &&
	       (uintptr_t)addr + len <= r->base + r->len);

	p = syscall_mmap(addr, len, PROT_READ | PROT_WRITE,
			 MAP_SHARED | MAP_FIXED, r->fd,
			 (uintptr_t)addr - r->base);
	if ((intptr_t)p < 0)
		return NULL;
	on_runtime_map(r->name, addr, len, PROT_READ | PROT_WRITE,
		       MAP_SHARED | MAP_FIXED);
	return p;
}

/**
 * runtime_mem_region_release - returns a slice's memory, keeping its mapping
 * @r: the region
 * @addr: the slice
 * @len: bytes
 *
 * Neither munmap nor MADV_DONTNEED frees a page of a shared memfd mapping;
 * only a hole punch does, and on a shared mapping MADV_REMOVE is exactly that
 * (madvise_remove -> vfs_fallocate PUNCH_HOLE). The mapping and the binding
 * stay, so the next use of this address needs no new binding and reads zeros.
 *
 * Returns 0 if successful, otherwise -errno.
 */
int runtime_mem_region_release(struct runtime_mem_region *r, void *addr,
			       size_t len)
{
	assert(r->fd >= 0);
	return syscall_madvise(addr, len, MADV_REMOVE);
}

/* libc conflicts with linux/shm.h, so define these ourselves */
void* shmat(int shm_id, const void *addr, int flags);
int shmctl(int shm_id, int cmd, struct shmid_ds* buf);
int shmdt(const void *addr);
int shmget(key_t key, size_t size, int flags);

void touch_mapping(void *base, size_t len, size_t pgsize)
{
	char *pos;

	/*
	 * Unfortunately mmap() provides no error message if MAP_POPULATE fails
	 * because of insufficient memory. Therefore, we manually force a write
	 * on each page to make sure the mapping was successful.
	 */
	for (pos = (char *)base; pos < (char *)base + len; pos += pgsize)
		ACCESS_ONCE(*pos);
}

static void *
__mem_map_anom(void *base, size_t len, size_t pgsize,
	       unsigned long *mask, int numa_policy)
{
	void *addr;
	/*
	 * Junction runs guest processes in separate address spaces, each one
	 * created by cloning the host process. The runtime's own memory must
	 * therefore be shared rather than copied, or the LibOS state would
	 * silently fork along with the guest. See junction/kernel/as.h.
	 */
	int flags = (cfg_shared_runtime_mem ? MAP_SHARED : MAP_PRIVATE) |
		    MAP_ANONYMOUS | MAP_POPULATE;

	len = align_up(len, pgsize);

	if (base)
		flags |= MAP_FIXED;

	switch (pgsize) {
	case PGSIZE_4KB:
	    break;
	case PGSIZE_2MB:
	    if (!cfg_transparent_hugepages_enabled) {
	        flags |= MAP_HUGETLB;
#ifdef MAP_HUGE_2MB
	        flags |= MAP_HUGE_2MB;
#endif
	    }
	    break;
	case PGSIZE_1GB:
#ifdef MAP_HUGE_1GB
	    if (!cfg_transparent_hugepages_enabled)
	        flags |= MAP_HUGETLB | MAP_HUGE_1GB;
#else
	    return MAP_FAILED;
#endif
	    break;
	default: /* fail on other sizes */
	  return MAP_FAILED;
	}

	addr = syscall_mmap(base, len, PROT_READ | PROT_WRITE, flags, -1, 0);
	if ((intptr_t)addr < 0)
		return MAP_FAILED;

  /*
	BUILD_ASSERT(sizeof(unsigned long) * 8 >= NNUMA);
	if (syscall_mbind(addr, len, numa_policy, mask ? mask : NULL,
		  mask ? NNUMA + 1 : 0, MPOL_MF_STRICT | MPOL_MF_MOVE))
		goto fail;
  */

	if (cfg_transparent_hugepages_enabled && (pgsize > PGSIZE_4KB)) {
	  if (syscall_madvise(addr, len, MADV_HUGEPAGE))
	    goto fail;
	}

	touch_mapping(addr, len, pgsize);
	on_runtime_map("mem_map_anom", addr, len, PROT_READ | PROT_WRITE, flags);
	return addr;

fail:
	munmap(addr, len);
	return MAP_FAILED;
}

/**
 * mem_map_anom - map anonymous memory pages
 * @base: the base address (or NULL for automatic)
 * @len: the length of the mapping
 * @pgsize: the page size
 * @node: the NUMA node
 *
 * Returns the base address, or MAP_FAILED if out of memory
 */
void *mem_map_anom(void *base, size_t len, size_t pgsize, int node)
{
	unsigned long mask = (1 << node);
	return __mem_map_anom(base, len, pgsize, &mask, MPOL_BIND);
}

/**
 * mem_map_file - maps a file into memory
 * @base: the address (or automatic if NULL)
 * @len: the length in bytes
 * @fd: the file descriptor
 * @offset: the offset inside the file
 *
 * Returns the address of the mapping or MAP_FAILED if failure.
 */
void *mem_map_file(void *base, size_t len, int fd, off_t offset)
{
	return mmap(base, len, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, offset);
}


static void *__mem_map_shm(mem_key_t key, void *base, size_t len,
		  size_t pgsize, bool exclusive, bool rdonly);

/**
 * mem_map_shm - maps a System V shared memory segment
 * @key: the unique key that identifies the shared region (e.g. use ftok())
 * @base: the base address to map the shared segment (or automatic if NULL)
 * @len: the length of the mapping
 * @pgsize: the size of each page
 * @exclusive: ensure this call creates the shared segment
 *
 * Returns a pointer to the mapping, or NULL if the mapping failed.
 */
void *mem_map_shm(mem_key_t key, void *base, size_t len, size_t pgsize,
		  bool exclusive)
{
	return __mem_map_shm(key, base, len, pgsize, exclusive, false);
}

void *mem_map_shm_rdonly(mem_key_t key, void *base, size_t len,
		  size_t pgsize)
{
	return __mem_map_shm(key, base, len, pgsize, false, true);
}

static void *__mem_map_shm(mem_key_t key, void *base, size_t len,
		  size_t pgsize, bool exclusive, bool rdonly)
{
	void *addr;
	int shmid, flags = rdonly ? 0 : (IPC_CREAT | 0744);

	BUILD_ASSERT(sizeof(mem_key_t) == sizeof(key_t));

	switch (pgsize) {
	case PGSIZE_4KB:
		break;
	case PGSIZE_2MB:
		flags |= SHM_HUGETLB;
#ifdef SHM_HUGE_2MB
		flags |= SHM_HUGE_2MB;
#endif
		break;
	case PGSIZE_1GB:
#ifdef SHM_HUGE_1GB
		flags |= SHM_HUGETLB | SHM_HUGE_1GB;
#else
		return MAP_FAILED;
#endif
		break;
	default: /* fail on other sizes */
		return MAP_FAILED;
	}

	if (exclusive)
		flags |= IPC_EXCL;

	shmid = shmget(key, len, flags);
	if (shmid == -1)
		return MAP_FAILED;

	flags = rdonly ? SHM_RDONLY : 0;
	addr = shmat(shmid, base, flags);
	if (addr == MAP_FAILED)
		return MAP_FAILED;

	touch_mapping(addr, len, pgsize);
	return addr;
}

/**
 * mem_unmap_shm - detach a shared memory mapping
 * @addr: the base address of the mapping
 *
 * Returns 0 if successful, otherwise fail.
 */
int mem_unmap_shm(void *addr)
{
	if (shmdt(addr) == -1)
		return -errno;
	return 0;
}

#define PAGEMAP_PGN_MASK	0x7fffffffffffffULL
#define PAGEMAP_FLAG_PRESENT	(1ULL << 63)
#define PAGEMAP_FLAG_SWAPPED	(1ULL << 62)
#define PAGEMAP_FLAG_FILE	(1ULL << 61)
#define PAGEMAP_FLAG_SOFTDIRTY	(1ULL << 55)

/**
 * mem_lookup_page_phys_addrs - determines the physical address of pages
 * @addr: a pointer to the start of the pages (must be @size aligned)
 * @len: the length of the mapping
 * @pgsize: the page size (4KB, 2MB, or 1GB)
 * @paddrs: a pointer store the physical addresses (of @nr elements)
 *
 * Returns 0 if successful, otherwise failure.
 */
int mem_lookup_page_phys_addrs(void *addr, size_t len,
			       size_t pgsize, physaddr_t *paddrs)
{
	uintptr_t pos;
	uint64_t tmp;
	int fd, i = 0, ret = 0;

	/*
	 * 4 KB pages could be swapped out by the kernel, so it is not
	 * safe to get a machine address. If we later decide to support
	 * 4KB pages, then we need to mlock() the page first.
	 */
	if (pgsize == PGSIZE_4KB)
		return -EINVAL;

	fd = open("/proc/self/pagemap", O_RDONLY);
	if (fd < 0)
		return -EIO;

	for (pos = (uintptr_t)addr; pos < (uintptr_t)addr + len;
	     pos += pgsize) {
		if (lseek(fd, pos / PGSIZE_4KB * sizeof(uint64_t), SEEK_SET) ==
		    (off_t)-1) {
			ret = -EIO;
			goto out;
		}
		if (read(fd, &tmp, sizeof(uint64_t)) <= 0) {
			ret = -EIO;
			goto out;
		}
		if (!(tmp & PAGEMAP_FLAG_PRESENT)) {
			ret = -ENODEV;
			goto out;
		}

		paddrs[i++] = (tmp & PAGEMAP_PGN_MASK) * PGSIZE_4KB;
	}

out:
	close(fd);
	return ret;
}
