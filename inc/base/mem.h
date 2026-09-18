/*
 * mem.h - memory management
 */

#pragma once

#include <base/types.h>

enum {
	PGSHIFT_4KB = 12,
	PGSHIFT_2MB = 21,
	PGSHIFT_1GB = 30,
};

enum {
	PGSIZE_4KB = (1 << PGSHIFT_4KB), /* 4096 bytes */
	PGSIZE_2MB = (1 << PGSHIFT_2MB), /* 2097152 bytes */
	PGSIZE_1GB = (1 << PGSHIFT_1GB), /* 1073741824 bytes */
};

extern bool cfg_transparent_hugepages_enabled;

#define PGMASK_4KB	(PGSIZE_4KB - 1)
#define PGMASK_2MB	(PGSIZE_2MB - 1)
#define PGMASK_1GB	(PGSIZE_1GB - 1)

/* page numbers */
#define PGN_4KB(la)	(((uintptr_t)(la)) >> PGSHIFT_4KB)
#define PGN_2MB(la)	(((uintptr_t)(la)) >> PGSHIFT_2MB)
#define PGN_1GB(la)	(((uintptr_t)(la)) >> PGSHIFT_1GB)

#define PGOFF_4KB(la)	(((uintptr_t)(la)) & PGMASK_4KB)
#define PGOFF_2MB(la)	(((uintptr_t)(la)) & PGMASK_2MB)
#define PGOFF_1GB(la)	(((uintptr_t)(la)) & PGMASK_1GB)

#define PGADDR_4KB(la)	(((uintptr_t)(la)) & ~((uintptr_t)PGMASK_4KB))
#define PGADDR_2MB(la)	(((uintptr_t)(la)) & ~((uintptr_t)PGMASK_2MB))
#define PGADDR_1GB(la)	(((uintptr_t)(la)) & ~((uintptr_t)PGMASK_1GB))

typedef unsigned long physaddr_t; /* physical addresses */
typedef unsigned long virtaddr_t; /* virtual addresses */

#ifndef MAP_FAILED
#define MAP_FAILED	((void *)-1)
#endif

typedef unsigned int mem_key_t;

extern bool cfg_shared_runtime_mem;
extern void on_runtime_map(const char *what, void *addr, size_t len, int prot,
			   int flags);

/*
 * A runtime memory region that any address space can repair on demand.
 *
 * Junction creates a guest address space by cloning the host process, so a
 * mapping made afterwards exists only where it was made. A region is backed
 * by one memfd at a fixed binding, offset = addr - base: the whole index space
 * is reserved PROT_NONE up front (address space, not memory), each slice is
 * mapped from the memfd on first use, and freeing punches the pages out of
 * the memfd but keeps the mapping and the binding. A fault at any address in
 * the region, from any address space, is repaired by Junction with that same
 * arithmetic, which is safe precisely because the binding never changes.
 */
struct runtime_mem_region {
	const char	*name;
	uintptr_t	base;
	size_t		len;
	size_t		granule;	/* what one fault repairs */
	int		fd;
};

extern struct runtime_mem_region runtime_lgpage_region;
extern struct runtime_mem_region runtime_stack_region;

/*
 * @memfd_flags: extra memfd_create() flags. MFD_HUGETLB | MFD_HUGE_2MB gives a
 * region whose pages are 2 MB hugetlb pages from the kernel's pool -- what
 * Caladan's large pages were before this, and what its slabs and network
 * buffers are sized for. Falls back to ordinary pages if the pool is missing.
 */
extern int runtime_mem_region_init(struct runtime_mem_region *r,
				   const char *name, uintptr_t base,
				   size_t len, size_t granule,
				   unsigned int memfd_flags);
extern void *runtime_mem_region_map(struct runtime_mem_region *r, void *addr,
				    size_t len);
extern int runtime_mem_region_release(struct runtime_mem_region *r,
				      void *addr, size_t len);
extern void *mem_map_anom(void *base, size_t len, size_t pgsize, int node);
extern void *mem_map_file(void *base, size_t len, int fd, off_t offset);
extern void *mem_map_shm(mem_key_t key, void *base, size_t len,
			 size_t pgsize, bool exclusive);
extern void *mem_map_shm_rdonly(mem_key_t key, void *base, size_t len,
			 size_t pgsize);
extern int mem_unmap_shm(void *base);
extern int mem_lookup_page_phys_addrs(void *addr, size_t len, size_t pgsize,
				      physaddr_t *maddrs);
extern void touch_mapping(void *base, size_t len, size_t pgsize);

static inline int
mem_lookup_page_phys_addr(void *addr, size_t pgsize, physaddr_t *paddr)
{
	return mem_lookup_page_phys_addrs(addr, pgsize, pgsize, paddr);
}
