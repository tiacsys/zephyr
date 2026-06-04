/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_fifo_loop.c Repeated multi-context FIFO stress tests
 */

#include "test_fifo.h"

#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN 4
#define LOOPS 32

static fdata_t data[LIST_LEN];
static struct k_fifo fifo;
static K_THREAD_STACK_DEFINE(tstack_loop, STACK_SIZE);
static struct k_thread tdata;
static struct k_sem end_sema;

/**
 * @brief Enqueue LIST_LEN items to a fifo via k_fifo_put().
 *
 * @details
 * Appends @p data[0..LIST_LEN-1] to the tail of @p pfifo one by one via
 * k_fifo_put(), leaving the fifo with LIST_LEN items in insertion order.
 *
 * @param pfifo Non-null pointer to an initialised fifo.
 *
 * @see k_fifo_put()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_put(struct k_fifo *pfifo)
{
	/**TESTPOINT: fifo put*/
	for (int i = 0; i < LIST_LEN; i++) {
		k_fifo_put(pfifo, (void *)&data[i]);
	}
}

/**
 * @brief Dequeue LIST_LEN items from a fifo and verify their identity and order.
 *
 * @details
 * Drains @p pfifo by calling k_fifo_get() with K_NO_WAIT for each of
 * @p data[0..LIST_LEN-1] and asserting pointer identity via zassert_equal(),
 * confirming FIFO ordering.
 *
 * @pre @p pfifo must have been populated by @c tfifo_put().
 *
 * @param pfifo Non-null pointer to a fifo containing @p data[0..LIST_LEN-1].
 *
 * @see k_fifo_get()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_get(struct k_fifo *pfifo)
{
	void *rx_data;

	/*get fifo data from "fifo_put"*/
	for (int i = 0; i < LIST_LEN; i++) {
		/**TESTPOINT: fifo get*/
		rx_data = k_fifo_get(pfifo, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data[i]);
	}
}

/*entry of contexts*/
/**
 * @brief ISR-context phase: drain then refill the fifo.
 *
 * @details
 * Executed via irq_offload() during one iteration of @c tfifo_read_write().
 * In ISR context, dequeues LIST_LEN items via @c tfifo_get() then immediately
 * re-enqueues LIST_LEN items via @c tfifo_put(), leaving the fifo ready for
 * the consumer thread.
 *
 * @param p Pointer to the fifo under test, cast to @c const @c void*.
 *
 * @see k_fifo_put(), k_fifo_get()
 *
 * @ingroup fifo_api_procedures
 */
static void tIsr_entry(const void *p)
{
	TC_PRINT("isr fifo get\n");
	tfifo_get((struct k_fifo *)p);
	TC_PRINT("isr fifo put ---> ");
	tfifo_put((struct k_fifo *)p);
}

/**
 * @brief Consumer-thread phase: drain then refill the fifo in two signalled stages.
 *
 * @details
 * Executed as the body of the consumer thread spawned by @c tfifo_read_write().
 * Performs two phases:
 * -# **Phase 1**: dequeues LIST_LEN items via @c tfifo_get(), then signals
 *    @c end_sema so the main thread knows the drain is complete.
 * -# **Phase 2**: re-enqueues LIST_LEN items via @c tfifo_put(), then signals
 *    @c end_sema again so the main thread can proceed to its own drain.
 *
 * @pre @p p1 must point to a fifo that has been re-filled by the ISR phase
 *      of @c tfifo_read_write().
 *
 * @param p1 Pointer to the fifo under test, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_fifo_put(), k_fifo_get()
 *
 * @ingroup fifo_api_procedures
 */
static void tThread_entry(void *p1, void *p2, void *p3)
{
	TC_PRINT("thread fifo get\n");
	tfifo_get((struct k_fifo *)p1);
	k_sem_give(&end_sema);
	TC_PRINT("thread fifo put ---> ");
	tfifo_put((struct k_fifo *)p1);
	k_sem_give(&end_sema);
}

/**
 * @brief Run one iteration of the multi-context fifo read-write cycle.
 *
 * @details
 * Exercises fifo operations across three execution contexts in a fixed sequence:
 * -# **Main thread**: enqueues LIST_LEN items via @c tfifo_put().
 * -# **ISR** (via irq_offload()): dequeues and re-enqueues via @c tIsr_entry().
 * -# **Consumer thread** (@c tThread_entry): dequeues (phase 1), signals,
 *    re-enqueues (phase 2), signals.
 * -# **Main thread**: waits for both consumer signals then dequeues and
 *    verifies the final batch via @c tfifo_get().
 *
 * @param pfifo Non-null pointer to an initialised, empty fifo.
 *
 * @see k_fifo_put(), k_fifo_get()
 *
 * @ingroup fifo_api_procedures
 */
/* fifo read write job */
static void tfifo_read_write(struct k_fifo *pfifo)
{
	k_sem_init(&end_sema, 0, 1);
	/**TESTPOINT: thread-isr-thread data passing via fifo*/
	k_tid_t tid = k_thread_create(&tdata, tstack_loop, STACK_SIZE,
				      tThread_entry, pfifo, NULL, NULL,
				      K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

	TC_PRINT("main fifo put ---> ");
	tfifo_put(pfifo);
	irq_offload(tIsr_entry, pfifo);
	k_sem_take(&end_sema, K_FOREVER);
	k_sem_take(&end_sema, K_FOREVER);

	TC_PRINT("main fifo get\n");
	tfifo_get(pfifo);
	k_thread_abort(tid);
	TC_PRINT("\n");
}

/**
 * @brief FIFO operations remain correct across LOOPS repeated multi-context
 * read-write cycles.
 *
 * @details
 * Runs LOOPS (32) iterations of @c tfifo_read_write(), each of which exercises
 * k_fifo_put() and k_fifo_get() across the main thread, an ISR, and a consumer
 * thread.  The test confirms that fifo state stays consistent -- pointer identity
 * holds throughout -- with no accumulated corruption or ordering error over the
 * full loop.
 *
 * @reqref{zep-srs-24-3}
 * @reqref{zep-srs-24-7}
 *
 * @see k_fifo_init(), k_fifo_put(), k_fifo_get()
 * @testid{TSPEC-FIFO-1CPU-003}
 * @draft
 */
ZTEST(fifo_api_1cpu, test_fifo_loop)
{
	/** @par Arrange
	 * -# Initialise @p fifo via k_fifo_init().
	 */
	k_fifo_init(&fifo);

	/** @par Act
	 * -# Repeat @c tfifo_read_write() for LOOPS (32) iterations, each
	 *    exercising the full three-context put/get cycle.
	 */

	/** @par Assert
	 * -# Each call to @c tfifo_read_write() completes without assertion
	 *    failure: @c tfifo_get() verifies pointer identity in every context
	 *    across all 32 iterations.
	 */
	for (int i = 0; i < LOOPS; i++) {
		TC_PRINT("* Pass data by fifo in loop %d\n", i);
		tfifo_read_write(&fifo);
	}
}
