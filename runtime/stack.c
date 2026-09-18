/*
 * stack.c - allocates and manages per-thread stacks
 */

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
 * Stacks live in a region any address space can repair on demand (see
 * runtime_mem_region in base/mem.h): the whole RUNTIME_MAX_THREADS index
 * space is reserved, and each stack is mapped from the region's memfd the
 * first time its address is handed out. stack_pos only ever moves forward,
 * so that is exactly once per address; freed stacks are recycled through
 * free_stacks with their memory punched out but their mapping kept.
 *
 * Stacks are unguarded. A guard page would split the region's mapping once
 * per stack, and cloning an address space copies every mapping, so guards
 * would trade a constant-time fork for one that slows with every stack ever
 * allocated. (Mapping only the usable half and leaving the guard half as the
 * PROT_NONE reservation would give guards back at that same cost.)
 */

static struct tcache *stack_tcache;
DEFINE_PERTHREAD(struct tcache_perthread, stack_pt);

static struct stack *stack_create(void *base)
{
	return (struct stack *)runtime_mem_region_map(&runtime_stack_region,
						      base, sizeof(struct stack));
}

/* WARNING: the contents of the stack may be lost after reclaiming. */
static void stack_reclaim(struct stack *s)
{
	int ret = runtime_mem_region_release(&runtime_stack_region, s->usable,
					     RUNTIME_STACK_SIZE);
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
	int ret = runtime_mem_region_init(&runtime_stack_region, "runtime-stacks",
					  STACK_BASE_ADDR,
					  (size_t)RUNTIME_MAX_THREADS *
						  sizeof(struct stack),
					  sizeof(struct stack), 0);
	if (ret)
		return ret;

	stack_tcache = tcache_create("runtime_stacks", &stack_tcache_ops,
				     TCACHE_DEFAULT_MAG_SIZE,
				     RUNTIME_STACK_SIZE);
	if (!stack_tcache)
		return -ENOMEM;
	return 0;
}
