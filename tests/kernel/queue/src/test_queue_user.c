/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_queue_user.c User-space queue API tests
 */

#include "test_queue.h"

#ifdef CONFIG_USERSPACE

#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN        5

static K_THREAD_STACK_DEFINE(child_stack, STACK_SIZE);
static struct k_thread child_thread;
static ZTEST_BMEM struct qdata qdata[LIST_LEN * 2];

/**
 * @brief Verify queue contents from user mode then consume all items including a canceled wait.
 *
 * @details
 * Runs as a user-mode child thread spawned by @c test_queue_supv_to_user.
 * Performs the following assertions in order:
 * -# zassert_false(k_queue_is_empty()) -- queue is non-empty on entry.
 * -# k_queue_peek_head() returns the item with data == 0 (first inserted).
 * -# k_queue_peek_tail() returns the item with data == (LIST_LEN*2)-1 (last inserted).
 * -# Dequeues all LIST_LEN*2 items via k_queue_get() with K_FOREVER in a loop,
 *    asserting for each that qd->data == i (sequential order) and, for allocated
 *    items, that qd->snode.next == NULL (snode was not modified by the queue).
 * -# zassert_true(k_queue_is_empty()) -- queue is empty after the drain loop.
 * -# One final k_queue_get() with K_FOREVER blocks until the supervisor calls
 *    k_queue_cancel_wait(); asserts the return value is NULL.
 *
 * Signals @p p2 (a @c k_sem) on exit so the supervisor thread can synchronise.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Pointer to the completion semaphore (@c k_sem), cast to @c void*.
 * @param p3 Unused.
 *
 * @see k_queue_is_empty(), k_queue_peek_head(), k_queue_peek_tail(), k_queue_get()
 *
 * @ingroup queue_procedures
 */
void child_thread_get(void *p1, void *p2, void *p3)
{
	struct qdata *qd;
	struct k_queue *q = p1;
	struct k_sem *sem = p2;

	zassert_false(k_queue_is_empty(q));
	qd = k_queue_peek_head(q);
	zassert_equal(qd->data, 0);
	qd = k_queue_peek_tail(q);
	zassert_equal(qd->data, (LIST_LEN * 2) - 1,
		      "got %d expected %d", qd->data, (LIST_LEN * 2) - 1);

	for (int i = 0; i < (LIST_LEN * 2); i++) {
		qd = k_queue_get(q, K_FOREVER);

		zassert_equal(qd->data, i);
		if (qd->allocated) {
			/* snode should never have been touched */
			zassert_is_null(qd->snode.next, NULL);
		}
	}


	zassert_true(k_queue_is_empty(q));

	/* This one gets canceled */
	qd = k_queue_get(q, K_FOREVER);
	zassert_is_null(qd, NULL);

	k_sem_give(sem);
}

/**
 * @brief Items enqueued by a supervisor thread are fully verified and drained
 * by a user-mode child thread, including a canceled blocking wait.
 *
 * @details
 * Enqueues LIST_LEN*2 items from supervisor mode, alternating between
 * k_queue_append() (for even indices, allocated=false) and k_queue_alloc_append()
 * (for odd indices, allocated=true), producing a sequence with data values
 * 0, 1, 2, ..., (LIST_LEN*2)-1.  A user-mode child thread (@c child_thread_get)
 * running at K_HIGHEST_THREAD_PRIO then verifies the queue head, tail, and full
 * dequeue order, and finally blocks on k_queue_get() with K_FOREVER.  The
 * supervisor cancels that wait, causing the child to receive NULL.  The child
 * signals a semaphore on exit to confirm completion without assertion failure.
 * Skipped if CONFIG_USERSPACE is not enabled.
 *
 * @reqref{zep-srs-20-3}
 * @reqref{zep-srs-20-6}
 * @reqref{zep-srs-20-8}
 * @reqref{zep-srs-20-9}
 * @reqref{zep-srs-20-14}
 *
 * @see k_queue_init(), k_queue_append(), k_queue_alloc_append(),
 *      k_queue_is_empty(), k_queue_peek_head(), k_queue_peek_tail(),
 *      k_queue_get(), k_queue_cancel_wait()
 * @testid{TSPEC-QUEUE-1CPU-006}
 * @draft
 */
ZTEST(queue_api_1cpu, test_queue_supv_to_user)
{
	struct k_queue *q;
	struct k_sem *sem;

	if (!(IS_ENABLED(CONFIG_USERSPACE))) {
		ztest_test_skip();
	}

	/** @par Arrange
	 * -# Allocate a kernel queue object via k_object_alloc() and initialise
	 *    it via k_queue_init().
	 * -# Allocate a kernel semaphore object via k_object_alloc() and
	 *    initialise it to 0 via k_sem_init().
	 * -# Enqueue LIST_LEN*2 items in pairs: for each pair, append
	 *    @p qdata[i] (allocated=false) via k_queue_append() and append
	 *    @p qdata[i+1] (allocated=true) via k_queue_alloc_append(), so
	 *    that data values 0..LIST_LEN*2-1 appear in the queue in order.
	 */
	q = k_object_alloc(K_OBJ_QUEUE);
	zassert_not_null(q, "no memory for allocated queue object");
	k_queue_init(q);

	sem = k_object_alloc(K_OBJ_SEM);
	zassert_not_null(sem, "no memory for semaphore object");
	k_sem_init(sem, 0, 1);

	for (int i = 0; i < (LIST_LEN * 2); i = i + 2) {
		/* Just for test purposes -- not safe to do this in the
		 * real world as user mode shouldn't have any access to the
		 * snode struct
		 */
		qdata[i].data = i;
		qdata[i].allocated = false;
		qdata[i].snode.next = NULL;
		k_queue_append(q, &qdata[i]);

		qdata[i + 1].data = i + 1;
		qdata[i + 1].allocated = true;
		qdata[i + 1].snode.next = NULL;
		zassert_false(k_queue_alloc_append(q, &qdata[i + 1]));
	}

	/** @par Act
	 * -# Create a user-mode child thread (@c child_thread_get) at
	 *    K_HIGHEST_THREAD_PRIO with K_USER | K_INHERIT_PERMS.
	 * -# Yield the supervisor thread so the child runs: it peeks, drains
	 *    all items, then blocks on a final k_queue_get() with K_FOREVER.
	 * -# Call k_queue_cancel_wait() to unblock the child's waiting get.
	 */
	k_thread_create(&child_thread, child_stack, STACK_SIZE,
			child_thread_get, q, sem, NULL, K_HIGHEST_THREAD_PRIO,
			K_USER | K_INHERIT_PERMS, K_NO_WAIT);

	k_yield();

	/* child thread runs until blocking on the last k_queue_get() call */
	k_queue_cancel_wait(q);

	/** @par Assert
	 * -# @c child_thread_get completes without assertion failure (verified
	 *    internally -- see its procedure docstring for the full list of
	 *    assertions) and signals @p sem.
	 * -# k_sem_take() on @p sem returns, confirming the child exited cleanly.
	 */
	k_sem_take(sem, K_FOREVER);
}

/**
 * @brief k_queue_alloc_prepend() produces LIFO order from user mode: items prepended
 * in ascending data order are dequeued in descending order.
 *
 * @details
 * From user mode, allocates a kernel queue object and prepends LIST_LEN*2 items
 * with data values 0, 1, ..., LIST_LEN*2-1 via k_queue_alloc_prepend().
 * Then dequeues all items with K_NO_WAIT and asserts that each returned item's
 * data value matches the expected LIFO index (LIST_LEN*2-1 down to 0), confirming
 * that prepend inserts at the head and get removes from the head.
 *
 * @reqref{zep-srs-20-4}
 * @reqref{zep-srs-20-6}
 * @reqref{zep-srs-20-14}
 *
 * @see k_queue_init(), k_queue_alloc_prepend(), k_queue_get()
 * @testid{TSPEC-QUEUE-API-008}
 * @draft
 */
ZTEST_USER(queue_api, test_queue_alloc_prepend_user)
{
	struct k_queue *q;

	/** @par Arrange
	 * -# Allocate a kernel queue object via k_object_alloc() and assert
	 *    it is non-null.
	 * -# Initialise the queue via k_queue_init().
	 */
	q = k_object_alloc(K_OBJ_QUEUE);
	zassert_not_null(q, "no memory for allocated queue object");
	k_queue_init(q);

	/** @par Act
	 * -# Prepend LIST_LEN*2 items in ascending data order (0, 1, ...,
	 *    LIST_LEN*2-1) via k_queue_alloc_prepend(), asserting each call
	 *    returns 0 (success).
	 */
	for (int i = 0; i < LIST_LEN * 2; i++) {
		qdata[i].data = i;
		zassert_false(k_queue_alloc_prepend(q, &qdata[i]));
	}

	/** @par Assert
	 * -# Dequeue all LIST_LEN*2 items via k_queue_get() with K_NO_WAIT.
	 *    For each index i counting down from LIST_LEN*2-1 to 0: the returned
	 *    pointer is non-null and qd->data == i, confirming LIFO ordering.
	 */
	for (int i = (LIST_LEN * 2) - 1; i >= 0; i--) {
		struct qdata *qd;

		qd = k_queue_get(q, K_NO_WAIT);
		zassert_true(qd != NULL);
		zassert_equal(qd->data, i);
	}
}

/**
 * @brief k_queue_alloc_append() produces FIFO order from user mode: items appended
 * in ascending data order are dequeued in the same order.
 *
 * @details
 * From user mode, allocates a kernel queue object and appends LIST_LEN*2 items
 * with data values 0, 1, ..., LIST_LEN*2-1 via k_queue_alloc_append().
 * Then dequeues all items with K_NO_WAIT and asserts that each returned item's
 * data value matches the insertion index, confirming that append inserts at the
 * tail and get removes from the head (FIFO ordering).
 *
 * @reqref{zep-srs-20-3}
 * @reqref{zep-srs-20-6}
 * @reqref{zep-srs-20-14}
 *
 * @see k_queue_init(), k_queue_alloc_append(), k_queue_get()
 * @testid{TSPEC-QUEUE-API-006}
 * @draft
 */
ZTEST_USER(queue_api, test_queue_alloc_append_user)
{
	struct k_queue *q;

	/** @par Arrange
	 * -# Allocate a kernel queue object via k_object_alloc() and assert
	 *    it is non-null.
	 * -# Initialise the queue via k_queue_init().
	 */
	q = k_object_alloc(K_OBJ_QUEUE);
	zassert_not_null(q, "no memory for allocated queue object");
	k_queue_init(q);

	/** @par Act
	 * -# Append LIST_LEN*2 items in ascending data order (0, 1, ...,
	 *    LIST_LEN*2-1) via k_queue_alloc_append(), asserting each call
	 *    returns 0 (success).
	 */
	for (int i = 0; i < LIST_LEN * 2; i++) {
		qdata[i].data = i;
		zassert_false(k_queue_alloc_append(q, &qdata[i]));
	}

	/** @par Assert
	 * -# Dequeue all LIST_LEN*2 items via k_queue_get() with K_NO_WAIT.
	 *    For each index i from 0 to LIST_LEN*2-1: the returned pointer is
	 *    non-null and qd->data == i, confirming FIFO ordering.
	 */
	for (int i = 0; i < LIST_LEN * 2; i++) {
		struct qdata *qd;

		qd = k_queue_get(q, K_NO_WAIT);
		zassert_true(qd != NULL);
		zassert_equal(qd->data, i);
	}
}

/**
 * @brief Implicitly-allocated queue elements and kernel objects from the preceding
 * tests are automatically freed, leaving the test pool fully available.
 *
 * @details
 * Verifies that resources consumed by the preceding user-space tests
 * (two kernel objects allocated via k_object_alloc() and LIST_LEN alloc'd
 * queue elements) have been released: queue elements are auto-freed on
 * dequeue, and kernel objects are freed when all threads holding permissions
 * on them exit.  The check is indirect: four 64-byte blocks are allocated
 * from @c test_pool with K_FOREVER and each is asserted non-null.  If any
 * earlier allocation had not been released, the pool would be exhausted and
 * the assertion would fail.  Skipped if CONFIG_USERSPACE is not enabled.
 *
 * @reqref{zep-srs-20-14}
 *
 * @see k_heap_alloc(), k_heap_free()
 * @testid{TSPEC-QUEUE-API-002}
 * @draft
 */
ZTEST(queue_api, test_auto_free)
{
	void *b[4];
	int i;

	if (!(IS_ENABLED(CONFIG_USERSPACE))) {
		ztest_test_skip();
	}

	/** @par Arrange
	 * -# Confirm CONFIG_USERSPACE is enabled (otherwise the test is skipped).
	 */

	/** @par Act
	 * -# Allocate four 64-byte blocks from @c test_pool via k_heap_alloc()
	 *    with K_FOREVER, one at a time.
	 */

	/** @par Assert
	 * -# Each allocation returns a non-null pointer, confirming the pool
	 *    has sufficient free space and that all memory from the preceding
	 *    tests was released automatically.
	 */
	/* Ensure any resources requested by the previous test were released
	 * by allocating the entire pool. It would have allocated two kernel
	 * objects and five queue elements. The queue elements should be
	 * auto-freed when they are de-queued, and the objects when all
	 * threads with permissions exit.
	 */
	for (i = 0; i < 4; i++) {
		b[i] = k_heap_alloc(&test_pool, 64, K_FOREVER);
		zassert_true(b[i] != NULL, "memory not auto released!");
	}

	/** @par Teardown
	 * -# Free all four allocated blocks back to @c test_pool.
	 */
	for (i = 0; i < 4; i++) {
		k_heap_free(&test_pool, b[i]);
	}
}

#endif /* CONFIG_USERSPACE */
