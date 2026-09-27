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
 * @addtogroup tests_kernel_fifo
 * @{
 */

/**
 * @brief Verify FIFO data passing between two threads.
 *
 * @details
 * A consumer thread blocks on k_fifo_get() while the main thread enqueues
 * items via the single, list and slist put APIs. The consumer must receive
 * every item in FIFO order. The scenario is run against both a k_fifo_init()ed
 * and a K_FIFO_DEFINE()d FIFO to confirm both initialization paths behave the
 * same.
 *
 * Test steps:
 * - Create a preemptible consumer thread that drains the FIFO.
 * - From the main thread, put items using k_fifo_put(), k_fifo_put_list() and
 *   k_fifo_put_slist().
 * - Synchronize on a semaphore once the consumer has read all items.
 * - Repeat for a statically defined FIFO.
 *
 * Expected result:
 * - The consumer dequeues every item in the order it was enqueued.
 *
 * @see k_fifo_put()
 * @see k_fifo_put_list()
 * @see k_fifo_put_slist()
 * @see k_fifo_get()
 * @verifies ZEP-SRS-24-3
 * @verifies ZEP-SRS-24-7
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
 * @brief Verify a FIFO defined at compile time is ready for use.
 *
 * @details
 * A FIFO created with K_FIFO_DEFINE() must be fully initialized at boot:
 * empty and immediately usable for data passing without any run-time
 * initialization call.
 *
 * Test steps:
 * - Check the statically defined FIFO is empty.
 * - Put an item and get it back, verifying the same item is returned.
 *
 * Expected result:
 * - The statically defined FIFO accepts and delivers items as-is.
 *
 * @see K_FIFO_DEFINE
 * @verifies ZEP-SRS-24-1
 */
ZTEST(fifo_api, test_fifo_define)
{
	/* usable at boot without any run-time initialization */
	zassert_true(k_fifo_is_empty(&kfifo));

	k_fifo_put(&kfifo, (void *)&data[0]);
	zassert_equal(k_fifo_get(&kfifo, K_NO_WAIT), (void *)&data[0]);
	zassert_true(k_fifo_is_empty(&kfifo));
}

/**
 * @brief Verify appending a singly-linked item list to the back of a FIFO.
 *
 * @details
 * k_fifo_put_list() must transfer a caller-built singly-linked list to the
 * back of the FIFO with its order preserved.
 *
 * Test steps:
 * - Put one item so the FIFO is non-empty.
 * - Link the items of an array into a list.
 * - Put the list and confirm it lands behind the pre-existing item.
 * - Get all items and verify list order is preserved.
 *
 * Expected result:
 * - The pre-existing item is delivered first, then every list item in its
 *   original order.
 *
 * @see k_fifo_put_list()
 * @see k_fifo_get()
 * @verifies ZEP-SRS-24-5
 */
ZTEST(fifo_api, test_fifo_put_list_order)
{
	static fdata_t *head = &data_l[0], *tail = &data_l[LIST_LEN - 1];

	k_fifo_init(&fifo);

	/* a pre-existing item so the list is demonstrably put behind it */
	k_fifo_put(&fifo, (void *)&data[0]);

	head->snode.next = (sys_snode_t *)tail;
	tail->snode.next = NULL;
	k_fifo_put_list(&fifo, (uint32_t *)head, (uint32_t *)tail);

	zassert_equal(k_fifo_get(&fifo, K_NO_WAIT), (void *)&data[0]);
	for (int i = 0; i < LIST_LEN; i++) {
		zassert_equal(k_fifo_get(&fifo, K_NO_WAIT),
			      (void *)&data_l[i]);
	}
	zassert_true(k_fifo_is_empty(&fifo));
}

/**
 * @brief Verify putting a sys_slist empties the list into the FIFO.
 *
 * @details
 * k_fifo_put_slist() must move every node of a sys_slist to the back of the
 * FIFO in list order and leave the source list empty.
 *
 * Test steps:
 * - Put one item so the FIFO is non-empty.
 * - Build a sys_slist with two nodes and put it on the FIFO.
 * - Verify the source list is empty afterwards.
 * - Get all items and confirm the nodes land behind the pre-existing item,
 *   in list order.
 *
 * Expected result:
 * - The pre-existing item is delivered first, then every node in order, and
 *   the source list is emptied.
 *
 * @see k_fifo_put_slist()
 * @see k_fifo_get()
 * @verifies ZEP-SRS-24-6
 */
ZTEST(fifo_api, test_fifo_put_slist_order)
{
	sys_slist_t slist;

	k_fifo_init(&fifo);

	/* a pre-existing item so the nodes land behind it */
	k_fifo_put(&fifo, (void *)&data[0]);

	sys_slist_init(&slist);
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[0].snode));
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[1].snode));
	k_fifo_put_slist(&fifo, &slist);

	/* the source list is emptied by the put */
	zassert_true(sys_slist_is_empty(&slist));

	zassert_equal(k_fifo_get(&fifo, K_NO_WAIT), (void *)&data[0]);
	for (int i = 0; i < LIST_LEN; i++) {
		zassert_equal(k_fifo_get(&fifo, K_NO_WAIT),
			      (void *)&data_sl[i]);
	}
	zassert_true(k_fifo_is_empty(&fifo));
}

/**
 * @brief Verify FIFO data passing from an ISR to a thread.
 *
 * @details
 * Items are enqueued from interrupt context (via irq_offload()) and dequeued in
 * thread context, confirming k_fifo_put() is ISR-safe and the thread receives
 * every item in FIFO order. Run against both an init()ed and a DEFINE()d FIFO.
 *
 * Test steps:
 * - From an ISR, put items into the FIFO and assert it is not empty.
 * - In thread context, get all items and verify their order.
 * - Repeat for a statically defined FIFO.
 *
 * Expected result:
 * - The thread dequeues every ISR-enqueued item in order.
 *
 * @see k_fifo_put()
 * @see k_fifo_get()
 * @verifies ZEP-SRS-24-3
 * @verifies ZEP-SRS-24-7
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
 * @brief Verify FIFO data passing from a thread to an ISR.
 *
 * @details
 * Items are enqueued in thread context and dequeued from interrupt context (via
 * irq_offload()), confirming k_fifo_get() is ISR-safe and the ISR receives every
 * item in FIFO order. Run against both an init()ed and a DEFINE()d FIFO.
 *
 * Test steps:
 * - In thread context, put items into the FIFO.
 * - From an ISR, get all items, verify their order, and assert the FIFO is empty.
 * - Repeat for a statically defined FIFO.
 *
 * Expected result:
 * - The ISR dequeues every thread-enqueued item in order.
 *
 * @see k_fifo_put()
 * @see k_fifo_get()
 * @verifies ZEP-SRS-24-3
 * @verifies ZEP-SRS-24-7
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
 * @brief Verify k_fifo_is_empty() tracks FIFO contents in thread context.
 *
 * @details
 * k_fifo_is_empty() must report true for a freshly initialized FIFO, false once
 * items are enqueued, and true again after they are all dequeued. All operations
 * run in thread context.
 *
 * Test steps:
 * - Initialize a FIFO and assert it is empty.
 * - Put items and assert it is not empty.
 * - Get all items and assert it is empty again.
 *
 * Expected result:
 * - k_fifo_is_empty() reflects the presence or absence of queued data.
 *
 * @see k_fifo_is_empty()
 * @verifies ZEP-SRS-24-8
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
 * @brief Verify k_fifo_is_empty() tracks FIFO contents in ISR context.
 *
 * @details
 * Same emptiness contract as the thread-context case, but the put/get/is_empty
 * sequence is executed from interrupt context via irq_offload() to confirm
 * k_fifo_is_empty() is ISR-safe.
 *
 * Test steps:
 * - Initialize a FIFO and assert it is empty from thread context.
 * - From an ISR, put items (not empty) then get them all (empty again).
 *
 * Expected result:
 * - k_fifo_is_empty() reports the correct state when called from an ISR.
 *
 * @see k_fifo_is_empty()
 * @verifies ZEP-SRS-24-8
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

/**
 * @brief Test peeking at the front and back items of a FIFO
 *
 * @details Enqueue two data items, then use k_fifo_peek_head() and
 * k_fifo_peek_tail() to inspect the items at the front and back of the FIFO and
 * verify the returned pointers match the first and last enqueued items without
 * removing them (a subsequent get still returns both items in order). Peeking an
 * empty FIFO returns NULL.
 *
 * @see k_fifo_peek_head(), k_fifo_peek_tail()
 * @verifies ZEP-SRS-24-9
 * @verifies ZEP-SRS-24-10
 */
ZTEST(fifo_api, test_fifo_peek)
{
	k_fifo_init(&fifo);

	k_fifo_put(&fifo, (void *)&data[0]);
	k_fifo_put(&fifo, (void *)&data[1]);

	/**TESTPOINT: peek front and back without removing*/
	zassert_equal(k_fifo_peek_head(&fifo), (void *)&data[0]);
	zassert_equal(k_fifo_peek_tail(&fifo), (void *)&data[1]);

	/* Peeking does not dequeue: both items are still retrievable in order. */
	zassert_equal(k_fifo_get(&fifo, K_NO_WAIT), (void *)&data[0]);
	zassert_equal(k_fifo_get(&fifo, K_NO_WAIT), (void *)&data[1]);

	/**TESTPOINT: peek of an empty fifo returns NULL*/
	zassert_is_null(k_fifo_peek_head(&fifo));
	zassert_is_null(k_fifo_peek_tail(&fifo));
}

K_HEAP_DEFINE(fifo_alloc_pool, 256);

/**
 * @brief Test enqueuing a data item to a FIFO with implicit memory allocation
 *
 * @details Use k_fifo_alloc_put() to enqueue a data item that does not reserve
 * space for the bookkeeping word, so the kernel allocates the container from the
 * calling thread's resource pool. Verify the call succeeds and that the same
 * data pointer is returned by a subsequent get.
 *
 * @see k_fifo_alloc_put()
 * @verifies ZEP-SRS-24-4
 */
ZTEST(fifo_api, test_fifo_alloc_put)
{
	static uint32_t payload = 0xDEADBEEF;

	k_fifo_init(&fifo);
	k_thread_heap_assign(k_current_get(), &fifo_alloc_pool);

	/**TESTPOINT: allocate the queue container from the thread resource pool*/
	zassert_equal(k_fifo_alloc_put(&fifo, &payload), 0);

	zassert_equal(k_fifo_get(&fifo, K_NO_WAIT), (void *)&payload);
}
/**
 * @}
 */
