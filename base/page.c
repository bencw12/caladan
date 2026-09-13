/*
 * page.c - the page allocator
 */

#include <sys/mman.h>

#include <base/slab.h>
#include <base/mem.h>
#include <base/page.h>
#include <base/lock.h>
#include <base/list.h>
#include <base/thread.h>
#include <base/cpu.h>
#include <base/log.h>
#include <base/init.h>
#include <base/tcache.h>

#include "init_internal.h"

/*
 * This pointer contains an array of page structs, organized as follows:
 * [NUMA 0 pages] [NUMA 1 pages] ... [NUMA N pages]
 */
struct page *page_tbl;

/* large page (2MB) definitions */
struct lgpage_node {
	spinlock_t		lock;
	unsigned int		idx;
	struct page		*tbl; /* aliases page_tbl above */
	struct list_head	pages;
	uint64_t		pad[4];
} __aligned(CACHE_LINE_SIZE);
static struct lgpage_node lgpage_nodes[NNUMA];

/* small page (4KB) definitions */
extern struct slab smpage_slab; /* defined in mm/slab.c */
extern struct tcache *smpage_tcache;
static DEFINE_PERTHREAD(struct tcache_perthread, smpage_pt);

#ifdef DEBUG

static void page_check(struct page *pg, size_t pgsize)
{
	/* since the page is allocated, it must be marked in use */
	assert(pg->flags & PAGE_FLAG_IN_USE);

	/* check for unsupported page sizes */
	assert(pgsize == PGSIZE_4KB || pgsize == PGSIZE_2MB);

	/* finally verify the page is configured correctly for its size */
	assert(page_to_size(pg) == pgsize);
	if (pgsize == PGSIZE_4KB) {
		assert(!(pg->flags & PAGE_FLAG_SHATTERED));
		pg = smpage_to_lgpage(pg);
		assert(pg->flags & PAGE_FLAG_LARGE);
		assert(pg->flags & PAGE_FLAG_SHATTERED);
	}

	/* check that the lgpage is inside the table */
	assert(pg - page_tbl >= 0 &&
	       pg - page_tbl < LGPAGE_META_ENTS * NNUMA);
}

static void page_alloc_check(struct page *pg, size_t pgsize)
{
	page_check(pg, pgsize);
	assert(!kref_released(&pg->ref));

	/* poison the page */
	memset(page_to_addr(pg), 0xEF, pgsize);
}

static void page_free_check(struct page *pg, size_t pgsize)
{
	page_check(pg, pgsize);
	assert(kref_released(&pg->ref));

	/* poison the page */
	memset(page_to_addr(pg), 0x89, pgsize);
}

#else /* DEBUG */

static void page_alloc_check(struct page *pg, size_t pgsize) {;}
static void page_free_check(struct page *pg, size_t pgsize) {;}

#endif /* DEBUG */

/*
 * How much of the large-page pool is reserved up front, in 2 MB units.
 *
 * Junction creates a guest address space by cloning the host process, so a
 * mapping made after a clone is only present in the address space that made
 * it. The runtime's heap therefore has to exist as one mapping before the
 * first clone rather than growing a mapping at a time. The reservation is
 * sparse, so this costs address space, not memory.
 */
#define LGPAGE_POOL_ENTRIES	512

static unsigned int nr_pooled_lgpages;

/*
 * Test affordance, mirroring stack.c. This pool backs *everything* Caladan
 * allocates dynamically -- uthread control blocks (thread_slab), mbufs,
 * smalloc, and the 4 KB page slab -- so overflowing it is a broader hazard
 * than the stack pool. Reaching it honestly takes 1 GB of live runtime
 * allocations; shrinking the reservation reaches the identical path sooner.
 */
static int lgpage_pool_entries(void)
{
	const char *e = getenv("JUNCTION_DEBUG_LGPAGE_POOL_ENTRIES");
	if (e) {
		int n = atoi(e);
		if (n >= 0)
			return n;
	}
	return LGPAGE_POOL_ENTRIES;
}

static int lgpage_create(struct page *pg, int numa_node)
{
	void *pgaddr = lgpage_to_addr(pg);

	if ((unsigned int)(pg - page_tbl) < nr_pooled_lgpages) {
		/* Already covered by the up-front reservation. */
		kref_init(&pg->ref);
		pg->flags = PAGE_FLAG_LARGE | PAGE_FLAG_IN_USE;
		return 0;
	}

	on_runtime_map("lgpage_create", pgaddr, PGSIZE_2MB,
		       PROT_READ | PROT_WRITE, 0);

	/*
	 * log_err, not log_warn: past this point Caladan's thread structs,
	 * mbufs and small allocations live in one address space only. The
	 * log_warn_ratelimited that used to be here never appeared in output
	 * even when the path was taken hundreds of times -- see
	 * docs/bug-stack-pool-overflow.md for the same problem in stack.c.
	 */
	if (nr_pooled_lgpages)
		log_err_ratelimited("page: growing the heap past the reserved pool; "
				    "new memory is visible only in the address "
				    "space that allocated it\n");

	pgaddr = mem_map_anom(pgaddr, PGSIZE_2MB, PGSIZE_2MB, numa_node);
	if (pgaddr == MAP_FAILED) {
		log_err_ratelimited("page: out of 2mb pages\n");
		return -ENOMEM;
	}

	kref_init(&pg->ref);
	pg->flags = PAGE_FLAG_LARGE | PAGE_FLAG_IN_USE;
	return 0;
}

static void lgpage_destroy(struct page *pg)
{
	/* Pooled pages belong to the up-front reservation; never unmap them. */
	if ((unsigned int)(pg - page_tbl) >= nr_pooled_lgpages)
		munmap(lgpage_to_addr(pg), PGSIZE_2MB);
	pg->flags = 0;
}

static struct page *lgpage_alloc_on_node(int numa_node)
{
	struct lgpage_node *node;
	struct page *pg;
	int ret;

	assert(numa_node < NNUMA);
	node = &lgpage_nodes[numa_node];

	spin_lock(&node->lock);
	pg = list_pop(&node->pages, struct page, link);
	if (!pg) {
		if (unlikely(node->idx >= LGPAGE_META_ENTS)) {
			spin_unlock(&node->lock);
			log_err_once("out of page region addresses");
			return NULL;
		}

		pg = &node->tbl[node->idx++];
	}
	spin_unlock(&node->lock);

	assert(!(pg->flags & PAGE_FLAG_IN_USE));
	ret = lgpage_create(pg, numa_node);
	if (ret) {
		log_err_once("page: unable to create 2MB page,"
			     "node = %d, ret = %d", numa_node, ret);
		return NULL;
	}

	return pg;
}

static void lgpage_free(struct page *pg)
{
	unsigned int numa_node = addr_to_numa_node(lgpage_to_addr(pg));
	struct lgpage_node *node = &lgpage_nodes[numa_node];

	assert(numa_node < NNUMA);
	lgpage_destroy(pg);
	spin_lock(&node->lock);
	list_add(&node->pages, &pg->link);
	spin_unlock(&node->lock);
}

static struct page *smpage_alloc_on_node(int numa_node)
{
	struct page *pg;
	void *addr;

	if (perthread_read(thread_init_done) && this_numa_node() == numa_node) {
		/* if on the local node use the fast path */
		addr = tcache_alloc(perthread_ptr(smpage_pt));
	} else {
		/* otherwise perform a remote slab allocation */
		addr = slab_alloc_on_node(&smpage_slab, numa_node);
	}

	if (!addr)
		return NULL;

	pg = addr_to_smpage(addr);
	kref_init(&pg->ref);
	pg->flags = PAGE_FLAG_IN_USE;
	return pg;
}

static void smpage_free(struct page *pg)
{
	void *addr = smpage_to_addr(pg);
	unsigned int numa_node = addr_to_numa_node(addr);

	assert(numa_node < NNUMA);
	pg->flags = 0;

	if (perthread_read(thread_init_done) && this_numa_node() == numa_node) {
		/* if on the local node use the fast path */
		tcache_free(perthread_ptr(smpage_pt), addr);
	} else {
		/* otherwise perform a remote slab free */
		slab_free(&smpage_slab, addr);
	}
}

/**
 * page_alloc_on_node - allocates a page for a NUMA node
 * @pgsize: the size of the page
 * @numa_node: the NUMA node the page is allocated from
 *
 * Returns a page, or NULL if an error occurred.
 */
struct page *page_alloc_on_node(size_t pgsize, int numa_node)
{
	struct page *pg;

	switch(pgsize) {
	case PGSIZE_4KB:
		pg = smpage_alloc_on_node(numa_node);
		break;
	case PGSIZE_2MB:
		pg = lgpage_alloc_on_node(numa_node);
		break;
	default:
		/* unsupported page size */
		pg = NULL;
	}

	page_alloc_check(pg, pgsize);
	return pg;
}

/**
 * page_alloc - allocates a page
 * @pgsize: the size of the page
 *
 * Returns a page, or NULL if out of memory.
 */
struct page *page_alloc(size_t pgsize)
{
	return page_alloc_on_node(pgsize, this_numa_node());
}

/**
 * page_zalloc - allocates a zeroed page
 * @pgsize: the size of the page
 *
 * Returns a page, or NULL if out of memory.
 */
struct page *page_zalloc(size_t pgsize)
{
	void *addr;
	struct page *pg = page_alloc(pgsize);
	if (!pg)
		return NULL;

	addr = page_to_addr(pg);
	memset(addr, 0, pgsize);
	return pg;
}

/**
 * page_alloc_addr_on_node - allocates a page address on for a NUMA node
 * @pgsize: the size of the page
 * @numa_node: the NUMA node the page is allocated from
 *
 * Returns a pointer to page data, or NULL if an error occurred.
 */
void *page_alloc_addr_on_node(size_t pgsize, int numa_node)
{
	struct page *pg = page_alloc_on_node(pgsize, numa_node);
	if (!pg)
		return NULL;
	return page_to_addr(pg);
}

/**
 * page_alloc_addr - allocates a page address
 * @pgsize: the size of the page
 *
 * Returns a pointer to page data, or NULL if an error occurred.
 */
void *page_alloc_addr(size_t pgsize)
{
	return page_alloc_addr_on_node(pgsize, this_numa_node());
}

/**
 * page_zalloc_addr_on_node - allocates a zeroed page address for a NUMA node
 * @pgsize: the size of the page
 * @numa_node: the NUMA node the page is allocated from
 *
 * Returns a pointer to zeroed page data, or NULL if an error occurred.
 */
void *page_zalloc_addr_on_node(size_t pgsize, int numa_node)
{
	void *addr = page_alloc_addr_on_node(pgsize, numa_node);
	if (addr)
		memset(addr, 0, pgsize);
	return addr;
}

/**
 * page_alloc_addr - allocates a page address
 * @pgsize: the size of the page
 *
 * Returns a pointer to zeroed page data, or NULL if an error occurred.
 */
void *page_zalloc_addr(size_t pgsize)
{
	void *addr = page_alloc_addr(pgsize);
	if (addr)
		memset(addr, 0, pgsize);
	return addr;
}

/**
 * page_put_addr - decrements underlying page's reference count
 * @addr: a pointer to the page data
 */
void page_put_addr(void *addr)
{
	assert(is_page_addr(addr));
	page_put(addr_to_page(addr));
}

/**
 * page_release - frees a page
 * @kref: the embedded kref struct inside a page
 */
void page_release(struct kref *ref)
{
	struct page *pg = container_of(ref, struct page, ref);
	size_t pgsize = page_to_size(pg);
	page_free_check(pg, pgsize);

	switch(pgsize) {
	case PGSIZE_4KB:
		smpage_free(pg);
		break;
	case PGSIZE_2MB:
		lgpage_free(pg);
		break;
	default:
		/* unsupported page size */
		panic("page: tried to free an invalid page size %ld", pgsize);
	}
}

/**
 * page_init - initializes the page subsystem
 */
int page_init(void)
{
	struct lgpage_node *node;
	void *addr;
	int i;

	/* First reserve address-space for the page table. */
	addr = mmap(NULL, LGPAGE_META_LEN * NNUMA + PGSIZE_2MB - 1, PROT_NONE,
		    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (addr == MAP_FAILED)
		return -ENOMEM;

	/* Align to the next 2MB boundary. */
	addr = (void *)align_up((uintptr_t)addr, PGSIZE_2MB);

	/* Then map NUMA-local large pages on top. */
	for (i = 0; i < 1; i++) {
		node = &lgpage_nodes[i];
		node->tbl = mem_map_anom(
			(char *)addr + i * LGPAGE_META_LEN,
			LGPAGE_META_NR_LGPAGES * PGSIZE_2MB, PGSIZE_2MB, i);
		if (node->tbl == MAP_FAILED)
			return -ENOMEM;

		spin_lock_init(&node->lock);
		list_head_init(&node->pages);
		node->idx = 0;
	}

	page_tbl = addr;

	if (cfg_shared_runtime_mem) {
		int lgentries = lgpage_pool_entries();
		size_t len = (size_t)lgentries * PGSIZE_2MB;
		void *p = mmap((void *)PAGE_BASE_ADDR, len,
			       PROT_READ | PROT_WRITE,
			       MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE |
			       MAP_FIXED_NOREPLACE, -1, 0);
		if (p != (void *)PAGE_BASE_ADDR) {
			log_err("page: could not reserve the large page pool");
			return -ENOMEM;
		}
		/* Ask for huge pages without requiring a hugetlb reservation. */
		madvise(p, len, MADV_HUGEPAGE);
		nr_pooled_lgpages = lgentries;
		log_info("page: reserved %d shared large pages (%ld MB)",
			 lgentries, len >> 20);
	}

	return 0;
}

/**
 * page_init_thread - initializes the page subsystem for a thread
 */
int page_init_thread(void)
{
	tcache_init_perthread(smpage_tcache, perthread_ptr(smpage_pt));
	return 0;
}
