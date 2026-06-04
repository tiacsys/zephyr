/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_fifo_contexts.c Context and is-empty tests for the FIFO API
 */

#include "test_fifo.h"

#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN 2
/**TESTPOINT: init via K_FIFO_DEFINE*/
K_FIFO_DEFINE(kfifo);

struct k_fifo fifo;
static fdata_t data[LIST_LEN];
static fdata_t data_l[LIST_LEN];
static fdata_t data_sl[LIST_LEN];

static K_THREAD_STACK_DEFINE(tstack_contexts, STACK_SIZE);
static struct k_thread tdata;
static struct k_sem end_sema;

/**
 * @brief Enqueue items exercising all three insertion APIs.
 *
 * @details
 * Populates @p pfifo in three passes:
 * -# k_fifo_put() -- enqueues @p data[0..LIST_LEN-1] one by one to the tail.
 * -# k_fifo_put_list() -- appends @p data_l[0..LIST_LEN-1] as a pre-linked
 *    singly-linked list.
 * -# k_fifo_put_slist() -- merges a @c sys_slist_t containing
 *    @p data_sl[0..LIST_LEN-1] onto the tail.
 *
 * After this procedure the fifo order (head to tail) is:
 * @p data[], @p data_l[], @p data_sl[].
 *
 * @param pfifo Non-null pointer to an initialised, empty fifo.
 *
 * @see k_fifo_put(), k_fifo_put_list(), k_fifo_put_slist()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_put(struct k_fifo *pfifo)
{
	for (int i = 0; i < LIST_LEN; i++) {
		/**TESTPOINT: fifo put*/
		k_fifo_put(pfifo, (void *)&data[i]);
	}

	/**TESTPOINT: fifo put list*/
	static fdata_t *head = &data_l[0], *tail = &data_l[LIST_LEN - 1];

	head->snode.next = (sys_snode_t *)tail;
	tail->snode.next = NULL;
	k_fifo_put_list(pfifo, (uint32_t *)head, (uint32_t *)tail);

	/**TESTPOINT: fifo put slist*/
	sys_slist_t slist;

	sys_slist_init(&slist);
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[0].snode));
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[1].snode));
	k_fifo_put_slist(pfifo, &slist);
}

/**
 * @brief Dequeue all items from a fifo and verify their identity and order.
 *
 * @details
 * Drains @p pfifo by calling k_fifo_get() with K_NO_WAIT for each expected
 * item and asserting pointer identity via zassert_equal().
 * The expected dequeue order, matching the insertion sequence of @c tfifo_put(), is:
 * -# @p data[0..LIST_LEN-1]   -- items inserted by k_fifo_put().
 * -# @p data_l[0..LIST_LEN-1] -- items inserted by k_fifo_put_list().
 * -# @p data_sl[0..LIST_LEN-1] -- items inserted by k_fifo_put_slist().
 *
 * @pre @p pfifo must have been populated by @c tfifo_put().
 *
 * @param pfifo Non-null pointer to a fifo populated by @c tfifo_put().
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
	/*get fifo data from "fifo_put_list"*/
	for (int i = 0; i < LIST_LEN; i++) {
		rx_data = k_fifo_get(pfifo, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data_l[i]);
	}
	/*get fifo data from "fifo_put_slist"*/
	for (int i = 0; i < LIST_LEN; i++) {
		rx_data = k_fifo_get(pfifo, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data_sl[i]);
	}
}

/*entry of contexts*/
/**
 * @brief ISR entry: enqueue all items and assert the fifo is non-empty.
 *
 * @details
 * Calls @c tfifo_put() to enqueue all three data arrays, then asserts via
 * zassert_false() that k_fifo_is_empty() returns false, confirming that
 * k_fifo_put() and the bulk insertion APIs are visible from ISR context.
 *
 * @param p Pointer to the fifo under test, cast to @c const @c void*.
 *
 * @see k_fifo_put(), k_fifo_put_list(), k_fifo_put_slist(), k_fifo_is_empty()
 *
 * @ingroup fifo_api_procedures
 */
static void tIsr_entry_put(const void *p)
{
	tfifo_put((struct k_fifo *)p);
	zassert_false(k_fifo_is_empty((struct k_fifo *)p));
}

/**
 * @brief ISR entry: dequeue all items and assert the fifo is empty.
 *
 * @details
 * Calls @c tfifo_get() to drain the fifo, then asserts via zassert_true()
 * that k_fifo_is_empty() returns true, confirming that k_fifo_get() fully
 * drains the fifo when called from ISR context.
 *
 * @param p Pointer to the fifo under test, cast to @c const @c void*.
 *
 * @see k_fifo_get(), k_fifo_is_empty()
 *
 * @ingroup fifo_api_procedures
 */
static void tIsr_entry_get(const void *p)
{
	tfifo_get((struct k_fifo *)p);
	zassert_true(k_fifo_is_empty((struct k_fifo *)p));
}

/** @cond INTERNAL */
static void tThread_entry(void *p1, void *p2, void *p3)
{
	tfifo_get((struct k_fifo *)p1);
	k_sem_give(&end_sema);
}
/** @endcond */

/**
 * @brief Run a thread-to-thread fifo transfer scenario and verify item delivery.
 *
 * @details
 * Creates a consumer thread at preemptive priority 0 that drains the fifo via
 * @c tThread_entry (which calls @c tfifo_get()), then enqueues 3*LIST_LEN items
 * from the current thread via @c tfifo_put().  Waits for the consumer thread to
 * signal completion via @c end_sema.  Item identity and order are verified inside
 * @c tfifo_get() by the consumer thread.
 *
 * @param pfifo Non-null pointer to an initialised, empty fifo.
 *
 * @see k_fifo_put(), k_fifo_get()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_thread_thread(struct k_fifo *pfifo)
{
	k_sem_init(&end_sema, 0, 1);
	/**TESTPOINT: thread-thread data passing via fifo*/
	k_tid_t tid = k_thread_create(&tdata, tstack_contexts, STACK_SIZE,
				      tThread_entry, pfifo, NULL, NULL,
				      K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	tfifo_put(pfifo);
	k_sem_take(&end_sema, K_FOREVER);
	k_thread_abort(tid);
}

/**
 * @brief Enqueue items from ISR context and dequeue them from thread context.
 *
 * @details
 * Uses irq_offload() to invoke @c tIsr_entry_put() in ISR context, which
 * enqueues 3*LIST_LEN items via all three insertion APIs and asserts the fifo
 * is non-empty.  The current thread then drains the fifo via @c tfifo_get(),
 * asserting pointer identity and order for every item.
 *
 * @param pfifo Non-null pointer to an initialised, empty fifo.
 *
 * @see k_fifo_put(), k_fifo_get()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_thread_isr(struct k_fifo *pfifo)
{
	k_sem_init(&end_sema, 0, 1);
	/**TESTPOINT: isr-thread data passing via fifo*/
	irq_offload(tIsr_entry_put, (const void *)pfifo);
	tfifo_get(pfifo);
}

/**
 * @brief Enqueue items from thread context and dequeue them from ISR context.
 *
 * @details
 * The current thread enqueues 3*LIST_LEN items via @c tfifo_put(), then uses
 * irq_offload() to invoke @c tIsr_entry_get() in ISR context, which drains the
 * fifo via @c tfifo_get() and asserts the fifo is empty on exit.
 *
 * @param pfifo Non-null pointer to an initialised, empty fifo.
 *
 * @see k_fifo_put(), k_fifo_get()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_isr_thread(struct k_fifo *pfifo)
{
	k_sem_init(&end_sema, 0, 1);
	/**TESTPOINT: thread-isr data passing via fifo*/
	tfifo_put(pfifo);
	irq_offload(tIsr_entry_get, (const void *)pfifo);
}

/**
 * @brief Enqueue then dequeue all items and assert k_fifo_is_empty() at each step.
 *
 * @details
 * Calls @c tfifo_put() on @p pfifo, asserts via zassert_false() that
 * k_fifo_is_empty() returns false (fifo is non-empty), then calls
 * @c tfifo_get() and asserts via zassert_true() that k_fifo_is_empty()
 * returns true (fifo is empty after draining).
 *
 * @param p Non-null pointer to an initialised fifo, cast to @c void*.
 *
 * @see k_fifo_put(), k_fifo_get(), k_fifo_is_empty()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_is_empty(void *p)
{
	struct k_fifo *pfifo = (struct k_fifo *)p;

	tfifo_put(&fifo);
	/**TESTPOINT: return false when data available*/
	zassert_false(k_fifo_is_empty(pfifo));

	tfifo_get(&fifo);
	/**TESTPOINT: return true with data unavailable*/
	zassert_true(k_fifo_is_empty(pfifo));
}

/**
 * @brief A fifo correctly transfers items between threads regardless of whether
 * it was initialised at runtime or at compile time.
 *
 * @details
 * Runs the full thread-to-thread transfer scenario twice: once on a fifo
 * initialised with k_fifo_init() and once on a fifo defined at compile time
 * with K_FIFO_DEFINE.  In both cases a consumer thread drains the fifo and
 * verifies item identity and order, confirming that the initialisation method
 * does not affect transfer correctness.
 *
 * @reqref{zep-srs-24-1}
 * @reqref{zep-srs-24-3}
 * @reqref{zep-srs-24-5}
 * @reqref{zep-srs-24-6}
 * @reqref{zep-srs-24-7}
 *
 * @see k_fifo_init(), k_fifo_put(), k_fifo_put_list(), k_fifo_put_slist(),
 *      k_fifo_get()
 * @testid{TSPEC-FIFO-1CPU-001}
 * @draft
 */
ZTEST(fifo_api_1cpu, test_fifo_thread2thread)
{
	/** @par Arrange
	 * -# Initialise @p fifo at runtime via k_fifo_init().
	 * -# @p kfifo is already available, having been defined at compile
	 *    time via K_FIFO_DEFINE.
	 */
	/**TESTPOINT: init via k_fifo_init*/
	k_fifo_init(&fifo);

	/** @par Act
	 * -# Run @c tfifo_thread_thread() on the runtime-initialised fifo.
	 * -# Run @c tfifo_thread_thread() on the compile-time-defined fifo.
	 */

	/** @par Assert
	 * -# Both invocations complete without assertion failure, confirming
	 *    that the consumer thread received all 3*LIST_LEN items in the
	 *    correct order (verified internally by @c tfifo_get()).
	 */
	tfifo_thread_thread(&fifo);

	/**TESTPOINT: test K_FIFO_DEFINEed fifo*/
	tfifo_thread_thread(&kfifo);
}

/**
 * @brief Items enqueued from ISR context are correctly delivered to the
 * dequeuing thread.
 *
 * @details
 * Runs the ISR-producer/thread-consumer scenario via @c tfifo_thread_isr()
 * on both a runtime-initialised fifo and a compile-time-defined fifo.  In
 * each case an ISR enqueues 3*LIST_LEN items using all three insertion APIs,
 * asserts the fifo is non-empty, and the current thread dequeues and verifies
 * them, confirming cross-context data passing regardless of initialisation.
 *
 * @reqref{zep-srs-24-3}
 * @reqref{zep-srs-24-5}
 * @reqref{zep-srs-24-6}
 * @reqref{zep-srs-24-7}
 *
 * @see k_fifo_init(), k_fifo_put(), k_fifo_put_list(), k_fifo_put_slist(),
 *      k_fifo_get()
 * @testid{TSPEC-FIFO-API-001}
 * @draft
 */
ZTEST(fifo_api, test_fifo_thread2isr)
{
	/** @par Arrange
	 * -# Initialise @p fifo at runtime via k_fifo_init().
	 * -# @p kfifo is available as a compile-time-defined fifo.
	 */
	/**TESTPOINT: init via k_fifo_init*/
	k_fifo_init(&fifo);

	/** @par Act
	 * -# Run @c tfifo_thread_isr() on the runtime-initialised fifo:
	 *    ISR enqueues items, current thread dequeues them.
	 * -# Run @c tfifo_thread_isr() on the compile-time-defined fifo.
	 */

	/** @par Assert
	 * -# Both invocations complete without assertion failure: @c tfifo_get()
	 *    verifies that every dequeued pointer matches the expected source
	 *    element in the correct order.
	 */
	tfifo_thread_isr(&fifo);

	/**TESTPOINT: test K_FIFO_DEFINEed fifo*/
	tfifo_thread_isr(&kfifo);
}

/**
 * @brief Items enqueued from thread context are correctly delivered to the
 * dequeuing ISR.
 *
 * @details
 * Runs the thread-producer/ISR-consumer scenario via @c tfifo_isr_thread()
 * on both a runtime-initialised fifo and a compile-time-defined fifo.  In
 * each case the current thread enqueues 3*LIST_LEN items using all three
 * insertion APIs, then an ISR drains and verifies them and asserts the fifo
 * is empty on exit.
 *
 * @reqref{zep-srs-24-3}
 * @reqref{zep-srs-24-5}
 * @reqref{zep-srs-24-6}
 * @reqref{zep-srs-24-7}
 *
 * @see k_fifo_init(), k_fifo_put(), k_fifo_put_list(), k_fifo_put_slist(),
 *      k_fifo_get()
 * @testid{TSPEC-FIFO-API-002}
 * @draft
 */
ZTEST(fifo_api, test_fifo_isr2thread)
{
	/** @par Arrange
	 * -# Initialise @p fifo at runtime via k_fifo_init().
	 * -# @p kfifo is available as a compile-time-defined fifo.
	 */
	/**TESTPOINT: test k_fifo_init fifo*/
	k_fifo_init(&fifo);

	/** @par Act
	 * -# Run @c tfifo_isr_thread() on the runtime-initialised fifo:
	 *    current thread enqueues items, ISR dequeues them.
	 * -# Run @c tfifo_isr_thread() on the compile-time-defined fifo.
	 */

	/** @par Assert
	 * -# Both invocations complete without assertion failure: @c tfifo_get()
	 *    (called from ISR context) verifies pointer identity and order, and
	 *    @c tIsr_entry_get() asserts the fifo is empty after draining.
	 */
	tfifo_isr_thread(&fifo);

	/**TESTPOINT: test K_FIFO_DEFINE fifo*/
	tfifo_isr_thread(&kfifo);
}

/**
 * @brief k_fifo_is_empty() correctly reflects the fifo state after init,
 * put, and get from thread context.
 *
 * @details
 * Verifies that k_fifo_is_empty() returns true immediately after
 * k_fifo_init(), false after @c tfifo_put() enqueues items, and true
 * again after @c tfifo_get() drains them -- all checked from thread context.
 *
 * @reqref{zep-srs-24-8}
 *
 * @see k_fifo_init(), k_fifo_is_empty(), k_fifo_put(), k_fifo_get()
 * @testid{TSPEC-FIFO-API-003}
 * @draft
 */
ZTEST(fifo_api, test_fifo_is_empty_thread)
{
	/** @par Arrange
	 * -# Initialise @p fifo via k_fifo_init().
	 */
	k_fifo_init(&fifo);

	/** @par Act
	 * -# Check k_fifo_is_empty() immediately after initialisation.
	 * -# Invoke @c tfifo_is_empty() which enqueues, checks, dequeues, and
	 *    checks again from thread context.
	 */

	/** @par Assert
	 * -# k_fifo_is_empty() returns true immediately after init.
	 * -# Inside @c tfifo_is_empty(): k_fifo_is_empty() returns false after
	 *    enqueue and true after the drain.
	 */
	/**TESTPOINT: k_fifo_is_empty after init*/
	zassert_true(k_fifo_is_empty(&fifo));

	/**TESTPONT: check fifo is empty from thread*/
	tfifo_is_empty(&fifo);
}

/**
 * @brief k_fifo_is_empty() correctly reflects the fifo state after put and
 * get from ISR context.
 *
 * @details
 * Runs @c tfifo_is_empty() via irq_offload() so that all enqueue, empty-check,
 * dequeue, and empty-check operations execute in ISR context, confirming that
 * k_fifo_is_empty() is accurate when called from an interrupt handler.
 *
 * @reqref{zep-srs-24-8}
 *
 * @see k_fifo_init(), k_fifo_is_empty(), k_fifo_put(), k_fifo_get()
 * @testid{TSPEC-FIFO-API-004}
 * @draft
 */
ZTEST(fifo_api, test_fifo_is_empty_isr)
{
	/** @par Arrange
	 * -# Initialise @p fifo via k_fifo_init().
	 */
	k_fifo_init(&fifo);

	/** @par Act
	 * -# Use irq_offload() to invoke @c tfifo_is_empty() in ISR context.
	 */

	/** @par Assert
	 * -# @c tfifo_is_empty() completes without assertion failure: it asserts
	 *    k_fifo_is_empty() returns false after enqueue and true after the
	 *    drain, all from ISR context.
	 */
	/**TESTPOINT: check fifo is empty from isr*/
	irq_offload((irq_offload_routine_t)tfifo_is_empty, &fifo);
}
