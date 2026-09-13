/*
 * stack.c - allocates and manages per-thread stacks
 */

#include <stdlib.h>
#include <sys/mman.h>

#include <base/stddef.h>
#include <base/lock.h>
#include <base/mem.h>
#include <base/page.h>
#include <base/atomic.h>
#include <base/limits.h>
#include <base/log.h>
#include <base/syscall.h>

#include "defs.h"

/*
 * Above the guest address range, in a 512 GB top-level slot of its own, so the
 * page tables for this region can be shared across guest address spaces. See
 * docs/shared-page-tables.md. Spans RUNTIME_MAX_THREADS * 1 MB = 100 GB.
 */
#define STACK_BASE_ADDR	0x520000000000UL

/*
 * Junction creates a guest address space by cloning the host process, so a
 * mapping made after a clone exists only in the address space that made it.
 * Stacks must therefore all exist before the first clone: the whole pool is
 * reserved up front as a single shared mapping, and stack_create() just carves
 * from it. The reservation is sparse -- pages are faulted in on demand, into
 * shared memory that every address space sees.
 *
 * The cost is the per-stack guard page. Turning one into PROT_NONE splits the
 * pool's mapping in two, and cloning an address space copies every mapping, so
 * keeping guards would trade a constant-time fork for one that slows down with
 * every stack ever allocated. Pooled stacks are unguarded.
 */
#define STACK_POOL_ENTRIES	16384

/*
 * Test affordance. The pool is what keeps stack allocation from mapping memory
 * into a single address space, so the only way to exercise the overflow path is
 * to exhaust it -- which normally takes 16384 concurrent uthreads. Shrinking
 * the reservation reproduces exactly the same failure at a testable size. See
 * docs/traces/stack_overflow_race.c and scripts/stack_overflow_test.sh.
 */
static int stack_pool_entries(void)
{
	const char *e = getenv("JUNCTION_DEBUG_STACK_POOL_ENTRIES");
	if (e) {
		int n = atoi(e);
		if (n >= 0)
			return n;
	}
	return STACK_POOL_ENTRIES;
}

static void *stack_pool_end;

static struct tcache *stack_tcache;
DEFINE_PERTHREAD(struct tcache_perthread, stack_pt);

static struct stack *stack_create(void *base)
{
	void *stack_addr;
	struct stack *s;

	if (stack_pool_end && base && base < stack_pool_end)
		return (struct stack *)base;	/* already mapped, no guard */

	on_runtime_map("stack_create", base, sizeof(struct stack),
		       PROT_READ | PROT_WRITE, 0);

	/*
	 * log_err, not log_warn: this is a correctness hazard, not a warning.
	 * Past this point a stack is mapped into one address space only, and a
	 * uthread later handed that stack from the free list while a different
	 * address space is loaded faults on its own stack -- with no frame to
	 * report the fault from. docs/traces/stack_overflow_race.c reproduces
	 * it. The previous log_warn_ratelimited here never appeared in output
	 * even when the path was taken 350 times.
	 */
	if (stack_pool_end)
		log_err_ratelimited("stack: high-water mark exceeded the reserved "
				    "pool of %d stacks; new stacks are visible "
				    "only in the address space that allocated "
				    "them\n", stack_pool_entries());

	/*
	 * Stacks must be shared, not copied, across address spaces: a thread
	 * that switches address space keeps running on the same stack, so a
	 * private copy would lose every frame pushed in the other space.
	 */
	stack_addr = syscall_mmap(base, sizeof(struct stack), PROT_READ | PROT_WRITE,
			  (cfg_shared_runtime_mem ? MAP_SHARED : MAP_PRIVATE) |
			  MAP_ANONYMOUS, -1, 0);
	if (stack_addr == MAP_FAILED)
		return NULL;

	s = (struct stack *)stack_addr;
	if (syscall_mprotect(s->guard, RUNTIME_GUARD_SIZE, PROT_NONE) == - 1) {
		munmap(stack_addr, sizeof(struct stack));
		return NULL;
	}

	return s;
}

/* WARNING: the contents of the stack may be lost after reclaiming. */
static void stack_reclaim(struct stack *s)
{
	int ret;

	/*
	 * MADV_DONTNEED does not release shared pages; MADV_REMOVE does, by
	 * punching a hole in the backing object, which is what the pooled
	 * stacks need.
	 */
	if (stack_pool_end && (void *)s < stack_pool_end)
		ret = syscall_madvise(s->usable, RUNTIME_STACK_SIZE, MADV_REMOVE);
	else
		ret = syscall_madvise(s->usable, RUNTIME_STACK_SIZE, MADV_DONTNEED);
	WARN_ON_ONCE(ret);
}

static DEFINE_SPINLOCK(stack_lock);
static int free_stack_count;
static struct stack *free_stacks[RUNTIME_MAX_THREADS];
static atomic64_t stack_pos = ATOMIC_INIT(STACK_BASE_ADDR);

static void stack_tcache_free(struct tcache *tc, int nr, void **items)
{
	int i;

	/* try to release the backing memory first */
	for (i = 0; i < nr; i++)
		stack_reclaim(stack_from_tcache_handle(items[i]));

	/* then make the stacks available for reallocation */
	spin_lock(&stack_lock);
	for (i = 0; i < nr; i++)
		free_stacks[free_stack_count++] = items[i];
	BUG_ON(free_stack_count >=
	       RUNTIME_MAX_THREADS + TCACHE_DEFAULT_MAG_SIZE);
	spin_unlock(&stack_lock);
}

static int stack_tcache_alloc(struct tcache *tc, int nr, void **items)
{
	void *base;
	int i = 0;
	struct stack *s;

	spin_lock(&stack_lock);
	while (free_stack_count && i < nr) {
		items[i++] = free_stacks[--free_stack_count];
	}
	spin_unlock(&stack_lock);


	for (; i < nr; i++) {
		base = (void *)atomic64_fetch_and_add(&stack_pos,
						      sizeof(struct stack));
		s = stack_create(base);
		if (unlikely(!s))
			goto fail;
		items[i] = stack_to_tcache_handle(s);
	}

	return 0;

fail:
	log_err_ratelimited("stack: failed to allocate stack memory");
	stack_tcache_free(tc, i, items);
	return -ENOMEM;
}

static const struct tcache_ops stack_tcache_ops = {
	.alloc	= stack_tcache_alloc,
	.free	= stack_tcache_free,
};

/**
 * stack_init_thread - intializes per-thread state
 * Returns 0 (always successful).
 */
int stack_init_thread(void)
{
	tcache_init_perthread(stack_tcache, perthread_ptr(stack_pt));
	return 0;
}

/**
 * runtime_stack_init - initializes the stack allocator
 * Returns 0 if successful, or -ENOMEM if out of memory.
 */
int runtime_stack_init(void)
{
	if (cfg_shared_runtime_mem) {
		int entries = stack_pool_entries();
		size_t len = (size_t)entries * sizeof(struct stack);
		void *p = syscall_mmap((void *)STACK_BASE_ADDR, len,
				       PROT_READ | PROT_WRITE,
				       MAP_SHARED | MAP_ANONYMOUS |
				       MAP_NORESERVE | MAP_FIXED_NOREPLACE,
				       -1, 0);
		if ((intptr_t)p < 0 || p != (void *)STACK_BASE_ADDR) {
			log_err("stack: could not reserve the stack pool");
			return -ENOMEM;
		}
		stack_pool_end = (char *)STACK_BASE_ADDR + len;
		log_info("stack: reserved %d shared stacks (%ld MB of address space)",
			 entries, len >> 20);
	}

	stack_tcache = tcache_create("runtime_stacks", &stack_tcache_ops,
				     TCACHE_DEFAULT_MAG_SIZE,
				     RUNTIME_STACK_SIZE);
	if (!stack_tcache)
		return -ENOMEM;
	return 0;
}
