/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_queue_contexts.c Implementation of Kernel Queue API tests
 */

/**
 * @defgroup queue_api_procedures Shared Test Procedures
 * @ingroup queue_api
 * @brief Reusable helper procedures invoked by queue_api test cases.
 */

/**
 * @defgroup queue_api_1cpu_procedures Shared Test Procedures
 * @ingroup queue_api_1cpu
 * @brief Reusable helper procedures invoked by queue_api_1cpu test cases.
 */

#include "test_queue.h"

#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN   2
/**TESTPOINT: init via K_QUEUE_DEFINE*/
K_QUEUE_DEFINE(kqueue);

K_HEAP_DEFINE(mem_pool_fail, 8 + 128);
K_HEAP_DEFINE(mem_pool_pass, 64 * 4 + 128);

struct k_queue queue;
static qdata_t data[LIST_LEN];
static qdata_t data_p[LIST_LEN];
static qdata_t data_l[LIST_LEN];
static qdata_t data_sl[LIST_LEN];

static qdata_t *data_append;
static qdata_t *data_prepend;

static K_THREAD_STACK_DEFINE(tstack, STACK_SIZE);
static struct k_thread tdata;
static K_THREAD_STACK_DEFINE(tstack1, STACK_SIZE);
static struct k_thread tdata1;
static K_THREAD_STACK_DEFINE(tstack2, STACK_SIZE);
static struct k_thread tdata2;
static struct k_sem end_sema;

/**
 * @brief Enqueue items exercising all five insertion APIs.
 *
 * @details
 * Populates @p pqueue using every insertion variant in turn:
 * -# k_queue_insert() — inserts @p data[0] at the tail.
 * -# k_queue_append() — appends @p data[1..LIST_LEN-1].
 * -# k_queue_prepend() — prepends @p data_p[] in reverse index order so
 *    that @p data_p[0] ends up at the head.
 * -# k_queue_append_list() — appends @p data_l[0..LIST_LEN-1] as a
 *    pre-linked singly-linked list.
 * -# k_queue_merge_slist() — merges a @c sys_slist_t containing
 *    @p data_sl[0..LIST_LEN-1] onto the tail.
 *
 * After this procedure the queue contains 4*LIST_LEN items in the order:
 * @p data_p[], @p data[], @p data_l[], @p data_sl[].
 * This is exactly the order that @c tqueue_get() is designed to verify.
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_insert(), k_queue_append(), k_queue_prepend(),
 *      k_queue_append_list(), k_queue_merge_slist()
 *
 * @ingroup queue_api_procedures
 * @ingroup queue_api_1cpu_procedures
 */
static void tqueue_append(struct k_queue *pqueue)
{
	k_queue_insert(pqueue, k_queue_peek_tail(pqueue), (void *)&data[0]);

	for (int i = 1; i < LIST_LEN; i++) {
		/**TESTPOINT: queue append */
		k_queue_append(pqueue, (void *)&data[i]);
	}

	for (int i = LIST_LEN - 1; i >= 0; i--) {
		/**TESTPOINT: queue prepend */
		k_queue_prepend(pqueue, (void *)&data_p[i]);
	}

	/**TESTPOINT: queue append list*/
	static qdata_t *head = &data_l[0], *tail = &data_l[LIST_LEN - 1];

	head->snode.next = (sys_snode_t *)tail;
	tail->snode.next = NULL;
	k_queue_append_list(pqueue, (uint32_t *)head, (uint32_t *)tail);

	/**TESTPOINT: queue merge slist*/
	sys_slist_t slist;

	sys_slist_init(&slist);
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[0].snode));
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[1].snode));
	k_queue_merge_slist(pqueue, &slist);
}

/**
 * @brief Dequeue all items from a queue and verify their identity and order.
 *
 * @details
 * Drains @p pqueue by calling k_queue_get() with @c K_NO_WAIT for each
 * expected item and asserts pointer identity via zassert_equal().
 * The expected dequeue order, matching the insertion sequence of
 * @c tqueue_append(), is:
 * -# @p data_p[0..LIST_LEN-1] — items inserted by k_queue_prepend().
 * -# @p data[0..LIST_LEN-1]   — items inserted by k_queue_insert() /
 *    k_queue_append().
 * -# @p data_l[0..LIST_LEN-1] — items inserted by k_queue_append_list().
 * -# @p data_sl[0..LIST_LEN-1] — items inserted by k_queue_merge_slist().
 *
 * @pre @p pqueue must have been populated by @c tqueue_append().
 *
 * @param pqueue Non-null pointer to a queue populated by @c tqueue_append().
 *
 * @see k_queue_get()
 *
 * @ingroup queue_api_procedures
 * @ingroup queue_api_1cpu_procedures
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
	/*get queue data from "queue_append_list"*/
	for (int i = 0; i < LIST_LEN; i++) {
		rx_data = k_queue_get(pqueue, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data_l[i]);
	}
	/*get queue data from "queue_merge_slist"*/
	for (int i = 0; i < LIST_LEN; i++) {
		rx_data = k_queue_get(pqueue, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data_sl[i]);
	}
}

/*entry of contexts*/
/** @cond INTERNAL */
static void tIsr_entry_append(const void *p)
{
	tqueue_append((struct k_queue *)p);
}

static void tIsr_entry_get(const void *p)
{
	tqueue_get((struct k_queue *)p);
}
/** @endcond */

/** @cond INTERNAL */
static void tThread_entry(void *p1, void *p2, void *p3)
{
	tqueue_get((struct k_queue *)p1);
	k_sem_give(&end_sema);
}
/** @endcond */

/**
 * @brief Run a thread-to-thread queue transfer scenario and verify item delivery.
 *
 * @details
 * Creates a consumer thread at preemptive priority 0 that blocks on
 * k_queue_get() via @c tThread_entry, then calls @c tqueue_append() from
 * the current thread to enqueue 4*LIST_LEN items using all five insertion
 * APIs.  Waits for the consumer thread to drain the queue and signal
 * completion via a semaphore.  Item identity and order are verified inside
 * @c tqueue_get() by the consumer thread.
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_append(), k_queue_get(), tqueue_append(), tqueue_get()
 *
 * @ingroup queue_api_1cpu_procedures
 */
static void tqueue_thread_thread(struct k_queue *pqueue)
{
	k_sem_init(&end_sema, 0, 1);
	k_tid_t tid = k_thread_create(&tdata, tstack, STACK_SIZE, tThread_entry, pqueue, NULL, NULL,
				      K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	tqueue_append(pqueue);
	k_sem_take(&end_sema, K_FOREVER);
	k_thread_abort(tid);
}

/**
 * @brief Enqueue items from ISR context and dequeue them from thread context.
 *
 * @details
 * Uses irq_offload() to invoke @c tIsr_entry_append() in ISR context, which
 * populates @p pqueue with 4*LIST_LEN items via all five insertion APIs.
 * The current thread then drains the queue via @c tqueue_get(), asserting
 * pointer identity and order for every item.
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_append(), k_queue_get()
 *
 * @ingroup queue_api_procedures
 */
static void tqueue_thread_isr(struct k_queue *pqueue)
{
	k_sem_init(&end_sema, 0, 1);
	/**TESTPOINT: thread-isr data passing via queue*/
	irq_offload(tIsr_entry_append, (const void *)pqueue);
	tqueue_get(pqueue);
}

/**
 * @brief Enqueue items from thread context and dequeue them from ISR context.
 *
 * @details
 * The current thread populates @p pqueue with 4*LIST_LEN items via
 * @c tqueue_append(), then uses irq_offload() to invoke @c tIsr_entry_get()
 * in ISR context, which drains the queue via @c tqueue_get() and asserts
 * pointer identity and order for every item.
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_append(), k_queue_get()
 *
 * @ingroup queue_api_procedures
 */
static void tqueue_isr_thread(struct k_queue *pqueue)
{
	k_sem_init(&end_sema, 0, 1);
	/**TESTPOINT: isr-thread data passing via queue*/
	tqueue_append(pqueue);
	irq_offload(tIsr_entry_get, (const void *)pqueue);
}

/*test cases*/
/**
 * @brief A queue correctly transfers items between threads regardless of
 * whether it was initialised at runtime or at compile time.
 *
 * @details
 * Runs the full thread-to-thread transfer scenario twice: once on a queue
 * initialised with k_queue_init() and once on a queue defined at compile
 * time with K_QUEUE_DEFINE.  In both cases a consumer thread drains the
 * queue and verifies item identity and order, confirming that the
 * initialisation method does not affect transfer correctness.
 *
 * @see k_queue_init(), tqueue_thread_thread()
 * @testid{TSPEC-QUEUE-1CPU-007}
 * @draft
 */
ZTEST(queue_api_1cpu, test_queue_thread2thread)
{
	/** @par Arrange
	 * -# Initialise @p queue at runtime via k_queue_init().
	 * -# @p kqueue is already available, having been defined at compile
	 *    time via K_QUEUE_DEFINE and requiring no runtime initialisation.
	 */
	k_queue_init(&queue);

	/** @par Act
	 * -# Run @c tqueue_thread_thread() on the runtime-initialised queue.
	 * -# Run @c tqueue_thread_thread() on the compile-time-defined queue.
	 */

	/** @par Assert
	 * -# Both invocations of @c tqueue_thread_thread() complete without
	 *    assertion failure, confirming that the consumer thread received
	 *    all 4*LIST_LEN items in the correct order (verified internally
	 *    by @c tqueue_get()).
	 */
	tqueue_thread_thread(&queue);
	tqueue_thread_thread(&kqueue);
}

/**
 * @brief Items enqueued from ISR context are correctly delivered to the dequeuing thread.
 *
 * @details
 * Runs the ISR-producer/thread-consumer scenario via @c tqueue_thread_isr()
 * on both a runtime-initialised queue and a compile-time-defined queue.
 * In each case an ISR enqueues 4*LIST_LEN items using all five insertion APIs,
 * and the current thread dequeues and verifies them, confirming that cross-context
 * data passing works regardless of how the queue was initialised.
 *
 * @reqref{zep-srs-20-6}
 *
 * @see k_queue_init(), k_queue_insert(), k_queue_peek_tail(), k_queue_append(),
 *      k_queue_prepend(), k_queue_append_list(), k_queue_merge_slist(), k_queue_get()
 * @testid{TSPEC-QUEUE-API-018}
 * @draft
 */
ZTEST(queue_api, test_queue_thread2isr)
{
	/** @par Arrange
	 * -# Initialise @p queue at runtime via k_queue_init().
	 * -# @p kqueue is already available, having been defined at compile
	 *    time via K_QUEUE_DEFINE and requiring no runtime initialisation.
	 */
	/**TESTPOINT: init via k_queue_init*/
	k_queue_init(&queue);

	/** @par Act
	 * -# Run @c tqueue_thread_isr() on the runtime-initialised queue:
	 *    an ISR enqueues 4*LIST_LEN items, the current thread dequeues them.
	 * -# Run @c tqueue_thread_isr() on the compile-time-defined queue.
	 */

	/** @par Assert
	 * -# Both invocations of @c tqueue_thread_isr() complete without
	 *    assertion failure: @c tqueue_get() verifies that every dequeued
	 *    pointer matches the expected source element in the correct order.
	 */
	tqueue_thread_isr(&queue);
	tqueue_thread_isr(&kqueue);
}

/**
 * @brief Items enqueued from thread context are correctly delivered to the dequeuing ISR.
 *
 * @details
 * Runs the thread-producer/ISR-consumer scenario via @c tqueue_isr_thread()
 * on both a runtime-initialised queue and a compile-time-defined queue.
 * In each case the current thread enqueues 4*LIST_LEN items using all five
 * insertion APIs, then an ISR drains and verifies them, confirming that
 * thread-to-ISR data passing works regardless of initialisation method.
 *
 * @reqref{zep-srs-20-6}
 *
 * @see k_queue_init(), k_queue_insert(), k_queue_peek_tail(), k_queue_append(),
 *      k_queue_prepend(), k_queue_append_list(), k_queue_merge_slist(), k_queue_get()
 * @testid{TSPEC-QUEUE-API-014}
 * @draft
 */
ZTEST(queue_api, test_queue_isr2thread)
{
	/** @par Arrange
	 * -# Initialise @p queue at runtime via k_queue_init().
	 * -# @p kqueue is already available as a compile-time-defined queue.
	 */
	k_queue_init(&queue);

	/** @par Act
	 * -# Run @c tqueue_isr_thread() on the runtime-initialised queue:
	 *    the current thread enqueues 4*LIST_LEN items, an ISR dequeues them.
	 * -# Run @c tqueue_isr_thread() on the compile-time-defined queue.
	 */

	/** @par Assert
	 * -# Both invocations of @c tqueue_isr_thread() complete without
	 *    assertion failure: @c tqueue_get() (called from ISR context)
	 *    verifies that every dequeued pointer matches the expected source
	 *    element in the correct order.
	 */
	tqueue_isr_thread(&queue);
	tqueue_isr_thread(&kqueue);
}

/**
 * @brief Dequeue one item from a queue and assert it is non-null.
 *
 * @details
 * Blocks on k_queue_get() with K_FOREVER until an item is available,
 * then asserts via zassert_true() that the returned pointer is non-null,
 * confirming that a real data item was delivered.  Signals @c end_sema
 * after the assertion so the caller can synchronise on thread completion.
 *
 * @pre @p p1 must be a non-null pointer to an initialised @c k_queue.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_queue_get()
 *
 * @ingroup queue_api_1cpu_procedures
 */
static void tThread_get(void *p1, void *p2, void *p3)
{
	zassert_true(k_queue_get((struct k_queue *)p1, K_FOREVER) != NULL, NULL);
	k_sem_give(&end_sema);
}

/**
 * @brief Run a two-consumer scenario and verify each thread receives one item.
 *
 * @details
 * Creates two consumer threads at preemptive priority 0 that each block on
 * k_queue_get() with K_FOREVER via @c tThread_get.  After a 10 ms sleep to
 * let both threads initialise and enter their blocking wait, appends two items
 * to the queue.  Waits for both threads to signal completion via @c end_sema,
 * confirming that each thread received exactly one non-null item.  Both threads
 * are then aborted to clean up resources.
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_append(), k_queue_get()
 *
 * @ingroup queue_api_1cpu_procedures
 */
static void tqueue_get_2threads(struct k_queue *pqueue)
{
	k_sem_init(&end_sema, 0, 1);
	k_tid_t tid = k_thread_create(&tdata, tstack, STACK_SIZE, tThread_get, pqueue, NULL, NULL,
				      K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

	k_tid_t tid1 = k_thread_create(&tdata1, tstack1, STACK_SIZE, tThread_get, pqueue, NULL,
				       NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

	/* Wait threads to initialize */
	k_sleep(K_MSEC(10));

	k_queue_append(pqueue, (void *)&data[0]);
	k_queue_append(pqueue, (void *)&data[1]);
	/* Wait threads to finalize */
	k_sem_take(&end_sema, K_FOREVER);
	k_sem_take(&end_sema, K_FOREVER);

	k_thread_abort(tid);
	k_thread_abort(tid1);
}

/**
 * @brief Each of two threads waiting on a queue receives exactly one item when
 * two items are appended.
 *
 * @details
 * Two consumer threads block on k_queue_get() with K_FOREVER before any items
 * are present in the queue.  Two items are then appended in sequence, and the
 * test waits for both threads to signal completion.  This verifies that
 * k_queue_get() dispatches a unique item to each waiter and does not deliver
 * the same item to more than one thread.
 *
 * @reqref{zep-srs-20-6}
 *
 * @see k_queue_init(), k_queue_append(), k_queue_get()
 * @testid{TSPEC-QUEUE-1CPU-001}
 * @draft
 */
ZTEST(queue_api_1cpu, test_queue_get_2threads)
{
	/** @par Arrange
	 * -# Initialise @p queue at runtime via k_queue_init(), creating an
	 *    empty queue ready for concurrent dequeue testing.
	 */
	/**TESTPOINT: test k_queue_init queue*/
	k_queue_init(&queue);

	/** @par Act
	 * -# Invoke @c tqueue_get_2threads(), which spawns two consumer threads
	 *    that block on k_queue_get() with K_FOREVER, then appends two items
	 *    to the queue and waits for both threads to signal completion via
	 *    @c end_sema.
	 */

	/** @par Assert
	 * -# Both consumer threads (@c tThread_get) complete without assertion
	 *    failure: each thread asserts via zassert_true() that k_queue_get()
	 *    returned a non-null pointer, confirming that each waiting thread
	 *    received exactly one item and no item was delivered to more than
	 *    one waiter.
	 */
	tqueue_get_2threads(&queue);
}

/**
 * @brief Exercise implicit-allocation enqueue and verify graceful failure at resource limits.
 *
 * @details
 * Tests k_queue_alloc_append() and k_queue_alloc_prepend() across three
 * thread heap configurations:
 * -# **No heap**: k_queue_alloc_append() is called without a thread resource
 *    pool.  zassert_false() confirms k_queue_remove() returns false (item was
 *    never inserted due to allocation failure).
 * -# **Undersized heap** (@c mem_pool_fail): k_queue_alloc_prepend() is called
 *    with an exhausted pool.  zassert_false() confirms the item is absent and
 *    zassert_true(k_queue_is_empty()) confirms the queue is still empty.
 * -# **Sufficient heap** (@c mem_pool_pass): k_queue_alloc_prepend() returns 0
 *    (asserted via zassert_false()), the queue becomes non-empty
 *    (zassert_false(k_queue_is_empty())), and k_queue_get() returns non-null.
 *
 * @pre The caller must have exhausted @c mem_pool_fail before invoking this
 *      procedure so that it is truly full and no allocation can succeed.
 *
 * @param pqueue Non-null pointer to an initialised, empty queue.
 *
 * @see k_queue_alloc_append(), k_queue_alloc_prepend(), k_queue_remove(),
 *      k_queue_is_empty(), k_queue_get(), k_thread_heap_assign()
 *
 * @ingroup queue_api_procedures
 */
static void tqueue_alloc(struct k_queue *pqueue)
{
	k_thread_heap_assign(k_current_get(), NULL);

	/* Alloc append without resource pool */
	k_queue_alloc_append(pqueue, (void *)&data_append);

	/* Insertion fails and alloc returns NOMEM */
	zassert_false(k_queue_remove(pqueue, &data_append));

	/* Assign resource pool of lower size */
	k_thread_heap_assign(k_current_get(), &mem_pool_fail);

	/* Prepend to the queue, but fails because of
	 * insufficient memory
	 */
	k_queue_alloc_prepend(pqueue, (void *)&data_prepend);

	zassert_false(k_queue_remove(pqueue, &data_prepend));

	/* No element must be present in the queue, as all
	 * operations failed
	 */
	zassert_true(k_queue_is_empty(pqueue));

	/* Assign resource pool of sufficient size */
	k_thread_heap_assign(k_current_get(), &mem_pool_pass);

	zassert_false(k_queue_alloc_prepend(pqueue, (void *)&data_prepend), NULL);

	/* Now queue shouldn't be empty */
	zassert_false(k_queue_is_empty(pqueue));

	zassert_true(k_queue_get(pqueue, K_FOREVER) != NULL, NULL);
}

/**
 * @brief k_queue_alloc_append/prepend() fail gracefully without a resource pool
 * and succeed when a sufficient pool is assigned.
 *
 * @details
 * Verifies that implicit-allocation enqueue operations respect thread heap
 * constraints: operations without a pool or with an exhausted pool leave the
 * queue empty, while a sufficiently sized pool allows successful insertion and
 * subsequent retrieval.
 *
 * @reqref{zep-srs-20-14}
 *
 * @see k_queue_init(), k_queue_alloc_append(), k_queue_alloc_prepend(),
 *      k_thread_heap_assign(), k_queue_is_empty(), k_queue_get(), k_queue_remove()
 * @testid{TSPEC-QUEUE-API-004}
 * @draft
 */
ZTEST(queue_api, test_queue_alloc)
{
	/** @par Arrange
	 * -# Exhaust @c mem_pool_fail by repeatedly allocating 1-byte blocks
	 *    until k_heap_alloc() returns NULL, ensuring the pool is truly full.
	 * -# Initialise @p queue via k_queue_init().
	 */
	/* The mem_pool_fail pool is supposed to be too small to
	 * succeed any allocations, but in fact with the heap backend
	 * there's some base minimal memory in there that can be used.
	 * Make sure it's really truly full.
	 */
	while (k_heap_alloc(&mem_pool_fail, 1, K_NO_WAIT) != NULL) {
	}

	k_queue_init(&queue);

	/** @par Act
	 * -# Invoke @c tqueue_alloc(), which exercises k_queue_alloc_append()
	 *    and k_queue_alloc_prepend() with no pool, an exhausted pool, and
	 *    a sufficient pool in sequence.
	 */

	/** @par Assert
	 * -# With no pool: k_queue_remove() returns false (insertion failed).
	 * -# With @c mem_pool_fail: k_queue_remove() returns false and
	 *    k_queue_is_empty() confirms the queue is still empty.
	 * -# With @c mem_pool_pass: k_queue_alloc_prepend() returns 0 (success),
	 *    k_queue_is_empty() returns false, and k_queue_get() returns non-null.
	 */
	tqueue_alloc(&queue);
}

/**
 * @brief Continuously dequeue items from a queue and assert each is non-null.
 *
 * @details
 * Loops indefinitely, calling k_queue_get() with K_FOREVER on each iteration.
 * Each returned value is asserted non-null via zassert_true() and the shared
 * item counter pointed to by @p p2 is incremented.  The thread runs until
 * aborted by the test case.  Two threads using this entry point compete for
 * items inserted by @c test_queue_poll_race to verify absence of the
 * historical CONFIG_POLL race condition.
 *
 * @pre @p p1 must be a non-null pointer to an initialised @c k_queue.
 * @pre @p p2 must be a non-null pointer to a volatile @c int counter
 *      initialised to zero.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Pointer to a volatile @c int item counter, cast to @c void*.
 * @param p3 Unused.
 *
 * @see k_queue_get()
 *
 * @ingroup queue_api_1cpu_procedures
 */
static void queue_poll_race_consume(void *p1, void *p2, void *p3)
{
	struct k_queue *q = p1;
	int *count = p2;

	while (true) {
		zassert_true(k_queue_get(q, K_FOREVER) != NULL);
		*count += 1;
	}
}

/**
 * @brief k_queue_get() does not spuriously return NULL when two waiting threads
 * race for items appended to the queue.
 *
 * @details
 * Reproduces the conditions for a historical CONFIG_POLL race: a lower-priority
 * waiter could be woken by an insert, a higher-priority waiter could steal the
 * item before the lower-priority waiter ran, and the lower-priority waiter would
 * then return NULL before its timeout expired.  Two consumer threads at different
 * priorities block with K_FOREVER; two items are appended; after yielding the
 * test verifies that the combined consumed count equals two with no spurious
 * NULL return.
 *
 * @reqref{zep-srs-20-6}
 *
 * @see k_queue_init(), k_queue_append(), k_queue_get()
 * @testid{TSPEC-QUEUE-1CPU-005}
 * @draft
 */
ZTEST(queue_api_1cpu, test_queue_poll_race)
{
	int prio = k_thread_priority_get(k_current_get());
	static volatile int mid_count, low_count;

	/** @par Arrange
	 * -# Initialise @p queue via k_queue_init().
	 * -# Create a mid-priority consumer thread (@c queue_poll_race_consume,
	 *    priority prio+1) that loops on k_queue_get() and increments
	 *    @c mid_count per item received.
	 * -# Create a low-priority consumer thread (@c queue_poll_race_consume,
	 *    priority prio+2) that loops on k_queue_get() and increments
	 *    @c low_count per item received.
	 * -# Sleep 10 ms so both threads enter their K_FOREVER blocking wait.
	 */
	k_queue_init(&queue);

	k_thread_create(&tdata, tstack, STACK_SIZE, queue_poll_race_consume, &queue,
			(void *)&mid_count, NULL, prio + 1, 0, K_NO_WAIT);

	k_thread_create(&tdata1, tstack1, STACK_SIZE, queue_poll_race_consume, &queue,
			(void *)&low_count, NULL, prio + 2, 0, K_NO_WAIT);

	/* Let them initialize and block */
	k_sleep(K_MSEC(10));

	/** @par Act
	 * -# Append @p data[0] and @p data[1] to the queue, making two items
	 *    available to the waiting consumer threads.
	 */
	/* Insert two items.  This will wake up both threads, but the
	 * higher priority thread (tdata1) might (if CONFIG_POLL)
	 * consume both.  The lower priority thread should stay
	 * asleep.
	 */
	k_queue_append(&queue, &data[0]);
	k_queue_append(&queue, &data[1]);

	/** @par Assert
	 * -# Immediately after the two appends: @c low_count == 0 and
	 *    @c mid_count == 0, confirming that no consumer has been scheduled
	 *    yet (the test thread holds the CPU at higher priority).
	 * -# After sleeping 10 ms (yielding CPU to both consumer threads):
	 *    @c low_count + @c mid_count == 2, confirming that both items were
	 *    delivered exactly once with no spurious NULL return to either waiter.
	 */
	zassert_true(low_count == 0);
	zassert_true(mid_count == 0);

	k_sleep(K_MSEC(10));

	zassert_true(low_count + mid_count == 2);

	/** @par Teardown
	 * -# Abort both consumer threads to release their stacks.
	 */
	k_thread_abort(&tdata);
	k_thread_abort(&tdata1);
}

#define QUEUE_NUM 10
/**
 * @brief Multiple independent queues can be initialised and operated
 * simultaneously without interference.
 *
 * @details
 * Creates QUEUE_NUM queue instances, exercises every insertion and removal
 * API on each one in turn, and verifies that the items are returned in the
 * correct order.  The test confirms that independently initialised queues
 * do not share state.
 *
 * @see k_queue_init(), tqueue_append(), tqueue_get()
 * @testid{TSPEC-QUEUE-API-003}
 * @draft
 */
ZTEST(queue_api, test_multiple_queues)
{
	static struct k_queue queues[QUEUE_NUM];

	/** @par Arrange
	 * -# Declare a static array of QUEUE_NUM (10) @c k_queue objects.
	 * -# Initialise each queue individually via k_queue_init().
	 */

	/** @par Act
	 * -# For each queue, invoke @c tqueue_append() to enqueue 4*LIST_LEN
	 *    items using all five insertion APIs.
	 */

	/** @par Assert
	 * -# For each queue, invoke @c tqueue_get() which dequeues all items
	 *    and asserts via zassert_equal() that each returned pointer matches
	 *    the expected source array element in the correct order.
	 * -# All QUEUE_NUM queues complete without assertion failure,
	 *    confirming independent operation.
	 */
	for (int i = 0; i < QUEUE_NUM; i++) {
		k_queue_init(&queues[i]);
		tqueue_append(&queues[i]);
		tqueue_get(&queues[i]);
	}
}

/**
 * @brief Attempt to access a k_queue object from user mode without permission.
 *
 * @details
 * Marks the expected kernel fault as valid via ztest_set_fault_valid(), then
 * calls k_queue_is_empty() on @c queue from a user-mode thread that has not
 * been granted permission on the queue object.  The kernel is expected to
 * generate an oops, which ztest intercepts as the valid expected fault.
 *
 * @pre Must be invoked as a user-mode thread created with @c K_USER that has
 *      not been granted object permission on @c queue.
 *
 * @param p1 Unused.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_queue_is_empty()
 *
 * @ingroup queue_api_procedures
 */
void user_access_queue_private_data(void *p1, void *p2, void *p3)
{
	ztest_set_fault_valid(true);
	/* try to access to private kernel data, will happen kernel oops */
	k_queue_is_empty(&queue);
}

/**
 * @brief A user-mode thread accessing a k_queue object without permission triggers a kernel fault.
 *
 * @details
 * Verifies the kernel object permission system: k_queue APIs store bookkeeping
 * information directly in queue buffers that are visible from user mode, so
 * access must be exclusively through system call interfaces.  A user-mode thread
 * that calls k_queue_is_empty() on a queue object it has not been granted
 * permission on must cause a kernel oops.  ztest intercepts the expected fault
 * via ztest_set_fault_valid() inside @c user_access_queue_private_data.
 *
 * @see k_queue_init(), k_queue_insert(), k_queue_peek_tail(), k_queue_is_empty()
 * @testid{TSPEC-QUEUE-API-001}
 * @draft
 */
ZTEST(queue_api, test_access_kernel_obj_with_priv_data)
{
	/** @par Arrange
	 * -# Initialise @p queue via k_queue_init().
	 * -# Insert one item via k_queue_insert() so the queue is non-empty.
	 */
	k_queue_init(&queue);
	k_queue_insert(&queue, k_queue_peek_tail(&queue), (void *)&data[0]);

	/** @par Act
	 * -# Create a user-mode thread (@c K_USER) running
	 *    @c user_access_queue_private_data, which calls k_queue_is_empty()
	 *    on @c queue without holding a permission grant on it.
	 */
	k_thread_create(&tdata, tstack, STACK_SIZE, user_access_queue_private_data, NULL, NULL,
			NULL, 0, K_USER, K_NO_WAIT);

	/** @par Assert
	 * -# The user-mode thread triggers a kernel oops that ztest intercepts
	 *    as a valid expected fault (set by ztest_set_fault_valid() inside
	 *    @c user_access_queue_private_data).  k_thread_join() returns after
	 *    the fault is handled, confirming the permission check fired.
	 */
	k_thread_join(&tdata, K_FOREVER);
}

/**
 * @brief Dequeue one item and assert its value equals 0xCCC.
 *
 * @details
 * Blocks on k_queue_get() with K_FOREVER, then asserts via zassert_true()
 * that the dereferenced value of the returned pointer equals 0xCCC, confirming
 * that this low-priority thread receives the last-dispatched item only after
 * both higher-priority waiters have been served.
 *
 * @pre @p p1 must be a non-null pointer to an initialised @c k_queue.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_queue_get()
 *
 * @ingroup queue_api_1cpu_procedures
 */
static void low_prio_wait_for_queue(void *p1, void *p2, void *p3)
{
	struct k_queue *q = p1;
	uint32_t *ret = NULL;

	ret = k_queue_get(q, K_FOREVER);
	zassert_true(*ret == 0xccc, "The low priority thread get the queue data failed lastly");
}

/**
 * @brief Dequeue one item and assert its value equals 0xAAA.
 *
 * @details
 * Blocks on k_queue_get() with K_FOREVER, then asserts via zassert_true()
 * that the dereferenced value equals 0xAAA, confirming that this thread --
 * the highest-priority waiter that has also waited the longest -- is
 * dispatched the first item appended.
 *
 * @pre @p p1 must be a non-null pointer to an initialised @c k_queue.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_queue_get()
 *
 * @ingroup queue_api_1cpu_procedures
 */
static void high_prio_t1_wait_for_queue(void *p1, void *p2, void *p3)
{
	struct k_queue *q = p1;
	uint32_t *ret = NULL;

	ret = k_queue_get(q, K_FOREVER);
	zassert_true(*ret == 0xaaa,
		     "The highest priority and waited longest get the queue data failed firstly");
}

/**
 * @brief Dequeue one item and assert its value equals 0xBBB.
 *
 * @details
 * Blocks on k_queue_get() with K_FOREVER, then asserts via zassert_true()
 * that the dereferenced value equals 0xBBB, confirming that this thread --
 * the highest-priority waiter that waited the shorter time -- is dispatched
 * the second item appended.
 *
 * @pre @p p1 must be a non-null pointer to an initialised @c k_queue.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_queue_get()
 *
 * @ingroup queue_api_1cpu_procedures
 */
static void high_prio_t2_wait_for_queue(void *p1, void *p2, void *p3)
{
	struct k_queue *q = p1;
	uint32_t *ret = NULL;

	ret = k_queue_get(q, K_FOREVER);
	zassert_true(*ret == 0xbbb,
		     "The higher priority and waited longer get the queue data failed secondly");
}

/**
 * @brief Queue dispatches items to waiting threads in priority-then-FIFO order.
 *
 * @details
 * When multiple threads are blocked waiting on an empty queue,
 * k_queue_append() wakes threads according to the Zephyr dispatch policy:
 * highest-priority thread first, and among threads of equal priority, the
 * one that has waited the longest (FIFO within a priority level).
 * Three consumer threads are created at two distinct priorities; a timed
 * gap between the two equal-priority threads establishes an unambiguous
 * wait order.  Three items are then appended one-by-one and each consumer
 * thread verifies it received the expected value.
 *
 * @reqref{zep-srs-20-6}
 * @reqref{zep-srs-20-7}
 *
 * @see k_queue_init(), k_queue_is_empty(), k_queue_append(), k_queue_get()
 * @testid{TSPEC-QUEUE-1CPU-004}
 * @draft
 */
ZTEST(queue_api_1cpu, test_queue_multithread_competition)
{
	int old_prio = k_thread_priority_get(k_current_get());
	int prio = 10;
	uint32_t test_data[3];

	/** @par Arrange
	 * -# Set the test thread priority to 10 so that consumer threads created
	 *    at lower numeric priority values cannot preempt the test thread
	 *    during setup.
	 * -# Initialize @p queue via k_queue_init() and verify it is empty.
	 * -# Assign three distinct marker values: @p test_data[0]=0xAAA,
	 *    @p test_data[1]=0xBBB, @p test_data[2]=0xCCC.
	 * -# Create a **low-priority** consumer thread at priority prio+4 (=14)
	 *    (@c low_prio_wait_for_queue) that blocks on k_queue_get() with
	 *    @c K_FOREVER.
	 * -# Create the **first high-priority** consumer thread at priority
	 *    prio+2 (=12) (@c high_prio_t1_wait_for_queue) that blocks on
	 *    k_queue_get() with @c K_FOREVER.
	 * -# Sleep 10 ms so the first high-priority thread is guaranteed to have
	 *    waited longer than the second (establishes wait-order within the
	 *    equal-priority group).
	 * -# Create the **second high-priority** consumer thread at priority
	 *    prio+2 (=12) (@c high_prio_t2_wait_for_queue) that blocks on
	 *    k_queue_get() with @c K_FOREVER.
	 * -# Sleep 50 ms to ensure all three threads have entered their blocking
	 *    wait before any item is appended.
	 */
	memset(test_data, 0, sizeof(test_data));
	k_thread_priority_set(k_current_get(), prio);
	k_queue_init(&queue);
	zassert_true(k_queue_is_empty(&queue) != 0, " Initializing queue failed");

	test_data[0] = 0xAAA;
	test_data[1] = 0xBBB;
	test_data[2] = 0xCCC;

	k_thread_create(&tdata, tstack, STACK_SIZE, low_prio_wait_for_queue, &queue, NULL, NULL,
			prio + 4, 0, K_NO_WAIT);

	k_thread_create(&tdata1, tstack1, STACK_SIZE, high_prio_t1_wait_for_queue, &queue, NULL,
			NULL, prio + 2, 0, K_NO_WAIT);

	k_sleep(K_MSEC(10));

	k_thread_create(&tdata2, tstack2, STACK_SIZE, high_prio_t2_wait_for_queue, &queue, NULL,
			NULL, prio + 2, 0, K_NO_WAIT);

	k_sleep(K_MSEC(50));

	/** @par Act
	 * -# Append @p test_data[0] (0xAAA) -- the highest-priority
	 *    longest-waiting thread (first high-priority thread) should be woken.
	 * -# Append @p test_data[1] (0xBBB) -- the remaining high-priority thread
	 *    (second high-priority thread) should be woken.
	 * -# Append @p test_data[2] (0xCCC) -- the low-priority thread should be
	 *    woken last.
	 */
	k_queue_append(&queue, &test_data[0]);
	k_queue_append(&queue, &test_data[1]);
	k_queue_append(&queue, &test_data[2]);

	/** @par Assert
	 * Each consumer thread asserts the received value internally via
	 * zassert_true():
	 * -# @c high_prio_t1_wait_for_queue: received value must equal 0xAAA
	 *    (highest priority, waited longest).
	 * -# @c high_prio_t2_wait_for_queue: received value must equal 0xBBB
	 *    (highest priority, waited shorter).
	 * -# @c low_prio_wait_for_queue: received value must equal 0xCCC
	 *    (lower priority, woken last).
	 *
	 * All three threads are joined to confirm that each thread received
	 * exactly one item and completed without assertion failure.
	 */
	k_thread_priority_set_adjoin(&tdata, K_FOREVER);
	k_thread_join(&tdata1, K_FOREVER);
	k_thread_join(&tdata2, K_FOREVER);

	/** @par Teardown
	 * Restore the test thread's original priority that was saved before the
	 * test modified it.
	 */
	k_thread_priority_set(k_current_get(), old_prio);
}

/**
 * @brief k_queue_unique_append() rejects a duplicate item already present in the queue.
 *
 * @details
 * Verifies that k_queue_unique_append() appends a new item and returns true,
 * rejects an item already present in the queue and returns false, and accepts
 * a different item and returns true again.
 *
 * @reqref{zep-srs-20-13}
 *
 * @see k_queue_init(), k_queue_unique_append()
 * @testid{TSPEC-QUEUE-API-019}
 * @draft
 */
ZTEST(queue_api, test_queue_unique_append)
{
	bool ret;

	/** @par Arrange
	 * -# Initialise @p queue via k_queue_init().
	 */
	k_queue_init(&queue);

	/** @par Act
	 * -# Append @p data[0] for the first time via k_queue_unique_append().
	 * -# Attempt to append @p data[0] a second time (duplicate of an item
	 *    already in the queue).
	 * -# Append @p data[1] (a distinct item) via k_queue_unique_append().
	 */

	/** @par Assert
	 * -# First append of @p data[0]: returns true (new item accepted).
	 * -# Second append of @p data[0]: returns false (duplicate rejected).
	 * -# Append of @p data[1]: returns true (new item accepted).
	 */
	ret = k_queue_unique_append(&queue, (void *)&data[0]);
	zassert_true(ret, "queue unique append failed");

	ret = k_queue_unique_append(&queue, (void *)&data[0]);
	zassert_false(ret, "queue unique append should fail");

	ret = k_queue_unique_append(&queue, (void *)&data[1]);
	zassert_true(ret, "queue unique append failed");
}
