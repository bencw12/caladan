/*
 * interruptible_wait.h - support for interrupting blocked threads
 */

#pragma once

#include <base/list.h>
#include <base/lock.h>
#include <runtime/thread.h>

#define PREPARED_FLAG		(1U << 6)
#define PREPARED_MASK		(PREPARED_FLAG - 1)
#define WAKER_VAL			(1 + PREPARED_FLAG)

// Returns true if this thread was interrupted.
// @th must be thread_self().
//
// Arming is idempotent: it adds WAKER_VAL only if PREPARED_FLAG is not already
// set. That keeps both halves of the protocol honest.
//
// Every caller returns early when this reports an interrupt, without parking
// and without registering with a waker -- so the WAKER_VAL it added is a credit
// nobody will ever subtract. Adding one per call let it accumulate: two rounds
// and interrupt_state exceeds 2*WAKER_VAL, after which a waker's subtraction in
// interruptible_wake() can never reach zero and never calls thread_ready(). The
// thread is then wakeable only by interrupts and hangs for good once they stop.
// Observed as istate=131 on a thread parked in a futex wait whose waiter had
// already been dequeued; see docs/bug-fork-storm-hang.md.
//
// The two obvious alternatives are both wrong, and were both measured:
//
//   - subtracting the credit after an early return, or declining to arm at all,
//     clears PREPARED_FLAG. But the flag is what tells an in-flight waker to
//     take the refcount path: interruptible_wake_test() readies the thread
//     unconditionally when it is not set, which readies a thread that is
//     already running -- BUG_ON(th->thread_ready) in thread_ready_prepare().
//     Declining to arm turned 7 hangs in 30 runs into 4 assertion failures.
//
// Leaving the flag set while refusing to double-count keeps wakers on the
// refcount path and keeps at most one credit outstanding.
static inline bool prepare_interruptible(thread_t *th)
{
	assert(th == thread_self());
	int8_t old = atomic8_read(&th->interrupt_state);
	while (true) {
		/* Already armed and uncollected: do not add a second credit. */
		if (old & PREPARED_FLAG)
			return (old & PREPARED_MASK) > 0;
		if (atomic8_cmpxchg_weak_relaxed(&th->interrupt_state, &old,
						 old + WAKER_VAL))
			return (old & PREPARED_MASK) > 0;
	}
}

// Called after enqueuing a signal to set the interrupt flag.
// Can only be called once, must be synchronized with signal lock.
static inline bool deliver_interrupt(thread_t *th)
{
	if (atomic8_fetch_and_add_relaxed(&th->interrupt_state, 1) > 0) {
		thread_ready(th);
		return true;
	}

	return false;
}

// Returns interrupt state.
// After wakeup (must be synchronized using waker lock):
// if state > 1: thread was interrupted, needs to be disarmed
// else if state > 0: both interrupt and wake occurred, no disarm needed
// if state == 0: thread was woken normally, no interrupt.
static inline int get_interruptible_status(const thread_t *th)
{
	return atomic8_read(&th->interrupt_state) & PREPARED_MASK;
}

// Wake a thread that is blocked pending a wake or interrupt.
// This thread must have called prepare_interruptible().
// Must have previously synchronized with a waker lock.
static inline void interruptible_wake_prepared(thread_t *th)
{
	if (atomic8_sub_and_fetch_relaxed(&th->interrupt_state, WAKER_VAL) == 0)
		thread_ready(th);
}

// Check if a thread was prepared to receive interrupts.
// This can only be called by a waker, which must have previously synchronized
// with a waker lock.
static inline bool check_prepared(const thread_t *th) {
	return (atomic8_read(&th->interrupt_state) & PREPARED_FLAG) != 0;
}

// Test whether or not a non-interrupt waker should call thread_ready
static inline bool interruptible_wake_test(thread_t *th)
{
	return !check_prepared(th) ||
	        atomic8_sub_and_fetch_relaxed(&th->interrupt_state, WAKER_VAL) == 0;
}

// Wake a thread that is blocked.
// The thread does not need to have been armed with prepare_interruptible().
// If the caller is certain that this thread was armed, it can call
// interruptible_wake_prepared() directly.
// Must have previously synchronized with a waker lock.
static inline void interruptible_wake(thread_t *th)
{
	if (interruptible_wake_test(th))
		thread_ready(th);
}

// Must be called with signal lock to synchronize with interrupt delivery.
static inline void reset_interruptible_state(thread_t *th)
{
	atomic8_write(&th->interrupt_state, 0);
}

// Mark this thread interrupted, typically used when the blocked signal mask
// is updated. Caller should synchronize with signal lock.
static inline void set_interrupt_state_interrupted()
{
	atomic8_write(&thread_self()->interrupt_state, 1);
}

// Check if a thread was interrupted.
// @th must be thread_self().
static inline bool thread_interrupted(const thread_t *th)
{
	assert(th == thread_self());
	return atomic8_read(&th->interrupt_state) > 0;
}
