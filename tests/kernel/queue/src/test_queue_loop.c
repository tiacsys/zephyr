/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_queue_loop.c Repeated multi-context queue stress tests
 */

#include "test_queue.h"

#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN 4
#define LOOPS 32

static qdata_t data[LIST_LEN];
static qdata_t data_p[LIST_LEN];
static qdata_t data_r[LIST_LEN];
static struct k_queue queue;
static K_THREAD_STACK_DEFINE(tstack, STACK_SIZE);
static struct k_thread tdata;
static struct k_sem end_sema;

/**
 * @brief Enqueue items using append, prepend, and prepend-for-removal.
 *
 * @details
 * Populates @p pqueue in three passes:
 * -# Appends @p data[0..LIST_LEN-1] to the tail via k_queue_append().
 * -# Prepends @p data_p[LIST_LEN-1..0] so that @p data_p[0] ends up directly
 *    ahead of the appended items.
 * -# Prepends @p data_r[LIST_LEN-1..0] to the very front so that @p data_r[0]
 *    is at the head, to be consumed later by @c tqueue_find_and_remove().
 *
 * After this procedure the queue order (head to tail) is:
 * @p data_r[], @p data_p[], @p data[].
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_append(), k_queue_prepend()
 *
 * @ingroup queue_procedures
 */
static void tqueue_append(struct k_queue *pqueue)
{
	/**TESTPOINT: queue append*/
	for (int i = 0; i < LIST_LEN; i++) {
		k_queue_append(pqueue, (void *)&data[i]);
	}

	/**TESTPOINT: queue prepend*/
	for (int i = LIST_LEN - 1; i >= 0; i--) {
		k_queue_prepend(pqueue, (void *)&data_p[i]);
	}

	/**TESTPOINT: queue find and remove*/
	for (int i = LIST_LEN - 1; i >= 0; i--) {
		k_queue_prepend(pqueue, (void *)&data_r[i]);
	}
}

/**
 * @brief Dequeue data_p[] then data[] items and verify their identity and order.
 *
 * @details
 * Drains the middle and tail portions of @p pqueue by calling k_queue_get()
 * with K_NO_WAIT for each expected item and asserting pointer identity via
 * zassert_equal().  Expects the queue to contain @p data_p[] at the head
 * followed by @p data[] (i.e., @p data_r[] must already have been removed
 * by @c tqueue_find_and_remove()):
 * -# @p data_p[0..LIST_LEN-1] -- items inserted by k_queue_prepend().
 * -# @p data[0..LIST_LEN-1]   -- items inserted by k_queue_append().
 *
 * @pre @p pqueue must have been populated by @c tqueue_append() and
 *      @p data_r[] must have already been removed by @c tqueue_find_and_remove().
 *
 * @param pqueue Non-null pointer to a queue in the above state.
 *
 * @see k_queue_get()
 *
 * @ingroup queue_procedures
 */
static void tqueue_get(struct k_queue *pqueue)
{
	void *rx_data;

	/*get queue data from "queue_prepend"*/
	for (int i = 0; i < LIST_LEN; i++) {
		/**TESTPOINT: queue get*/
		rx_data = k_queue_get(pqueue, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data_p[i]);
	}

	/*get queue data from "queue_append"*/
	for (int i = 0; i < LIST_LEN; i++) {
		/**TESTPOINT: queue get*/
		rx_data = k_queue_get(pqueue, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data[i]);
	}
}

/**
 * @brief Remove data_r[] items from the queue and assert each removal succeeds.
 *
 * @details
 * Calls k_queue_remove() for each element of @p data_r[] in index order
 * and asserts via zassert_true() that the return value is true, confirming
 * that the item was present and successfully removed from @p pqueue.
 *
 * @pre @p pqueue must contain all elements of @p data_r[], inserted by
 *      @c tqueue_append().
 *
 * @param pqueue Non-null pointer to a queue containing @p data_r[].
 *
 * @see k_queue_remove()
 *
 * @ingroup queue_procedures
 */
static void tqueue_find_and_remove(struct k_queue *pqueue)
{
	/*remove queue data from "queue_find_and_remove"*/
	for (int i = 0; i < LIST_LEN; i++) {
		/**TESTPOINT: queue find and remove*/
		zassert_true(k_queue_remove(pqueue, &data_r[i]));
	}
}

/*entry of contexts*/
/**
 * @brief ISR-context phase: remove, drain, and refill the queue.
 *
 * @details
 * Executed via irq_offload() during one iteration of @c tqueue_read_write().
 * In ISR context, performs the following sequence:
 * -# Removes @p data_r[] from the queue via @c tqueue_find_and_remove().
 * -# Drains @p data_p[] and @p data[] via @c tqueue_get().
 * -# Re-fills the queue with all three arrays via @c tqueue_append(), leaving
 *    the queue in @p data_r[], @p data_p[], @p data[] order (head to tail)
 *    ready for the consumer thread.
 *
 * @pre @p p must point to a queue in the state produced by the preceding
 *      main-thread @c tqueue_append(): @p data_r[] at head, @p data_p[] next,
 *      @p data[] at tail.
 *
 * @param p Pointer to the queue under test, cast to @c const @c void*.
 *
 * @see k_queue_append(), k_queue_get(), k_queue_remove()
 *
 * @ingroup queue_procedures
 */
static void tIsr_entry(const void *p)
{
	tqueue_find_and_remove((struct k_queue *)p);
	tqueue_get((struct k_queue *)p);
	tqueue_append((struct k_queue *)p);
}

/**
 * @brief Consumer-thread phase: drain then refill the queue in two signalled stages.
 *
 * @details
 * Executed as the body of the consumer thread spawned by @c tqueue_read_write().
 * Performs two phases:
 * -# **Phase 1**: removes @p data_r[] via @c tqueue_find_and_remove(), drains
 *    @p data_p[] and @p data[] via @c tqueue_get(), then signals @c end_sema
 *    so the main thread knows the drain is complete.
 * -# **Phase 2**: re-fills the queue via @c tqueue_append(), then signals
 *    @c end_sema again so the main thread can proceed to its own drain.
 *
 * @pre @p p1 must point to a queue in the state left by the preceding ISR phase
 *      of @c tqueue_read_write(): @p data_r[] at head, @p data_p[] next,
 *      @p data[] at tail.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_queue_append(), k_queue_get(), k_queue_remove()
 *
 * @ingroup queue_procedures
 */
static void tThread_entry(void *p1, void *p2, void *p3)
{
	tqueue_find_and_remove((struct k_queue *)p1);
	tqueue_get((struct k_queue *)p1);
	k_sem_give(&end_sema);
	tqueue_append((struct k_queue *)p1);
	k_sem_give(&end_sema);
}

/**
 * @brief Run one iteration of the multi-context queue read-write cycle.
 *
 * @details
 * Exercises queue operations across three execution contexts in a fixed sequence:
 * -# **Main thread**: enqueues all data arrays via @c tqueue_append() --
 *    queue order becomes @p data_r[], @p data_p[], @p data[] (head to tail).
 * -# **ISR** (via irq_offload()): removes @p data_r[], drains @p data_p[]
 *    and @p data[], then re-fills all three arrays via @c tIsr_entry().
 * -# **Consumer thread** (@c tThread_entry): removes @p data_r[], drains
 *    @p data_p[] and @p data[], signals phase-1 completion, re-fills all
 *    three arrays, then signals phase-2 completion.
 * -# **Main thread**: waits for both consumer-thread signals, removes
 *    @p data_r[], drains @p data_p[] and @p data[], then aborts the thread.
 *
 * Each transfer step asserts pointer identity and removal success, verifying
 * correctness across all three context paths in a single pass.
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_append(), k_queue_get(), k_queue_remove()
 *
 * @ingroup queue_procedures
 */
/* queue read write job */
static void tqueue_read_write(struct k_queue *pqueue)
{
	k_sem_init(&end_sema, 0, 1);
	/**TESTPOINT: thread-isr-thread data passing via queue*/
	k_tid_t tid = k_thread_create(&tdata, tstack, STACK_SIZE,
				      tThread_entry, pqueue, NULL, NULL,
				      K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

	tqueue_append(pqueue);
	irq_offload(tIsr_entry, (const void *)pqueue);
	k_sem_take(&end_sema, K_FOREVER);
	k_sem_take(&end_sema, K_FOREVER);

	tqueue_find_and_remove(pqueue);
	tqueue_get(pqueue);
	k_thread_abort(tid);
}

/*test cases*/
/**
 * @brief Queue operations remain correct across LOOPS repeated multi-context cycles.
 *
 * @details
 * Runs LOOPS (32) iterations of @c tqueue_read_write(), each of which
 * exercises enqueue, dequeue, and removal across the main thread, an ISR,
 * and a consumer thread.  The test confirms that queue state stays consistent
 * -- pointer identity and removal success hold throughout -- with no
 * accumulated corruption or ordering error over the full loop.
 *
 * @reqref{zep-srs-20-3}
 * @reqref{zep-srs-20-4}
 * @reqref{zep-srs-20-5}
 * @reqref{zep-srs-20-6}
 *
 * @see k_queue_init(), k_queue_append(), k_queue_prepend(),
 *      k_queue_get(), k_queue_remove()
 * @testid{TSPEC-QUEUE-1CPU-003}
 * @draft
 */
ZTEST(queue_api_1cpu, test_queue_loop)
{
	/** @par Arrange
	 * -# Initialise @p queue via k_queue_init().
	 */
	k_queue_init(&queue);

	/** @par Act
	 * -# Repeat @c tqueue_read_write() for LOOPS (32) iterations, each
	 *    exercising the full three-context enqueue/dequeue/remove cycle.
	 */

	/** @par Assert
	 * -# Each call to @c tqueue_read_write() completes without assertion
	 *    failure: pointer identity is verified by @c tqueue_get() in every
	 *    context, and removal success is verified by @c tqueue_find_and_remove()
	 *    in every context, across all 32 iterations.
	 */
	for (int i = 0; i < LOOPS; i++) {
		TC_PRINT("* Pass data by queue in loop %d\n", i);
		tqueue_read_write(&queue);
	}
}
