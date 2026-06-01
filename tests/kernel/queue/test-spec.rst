Queue Tests
###########

Test module at ``tests/kernel/queue`` — contains all queue test suites.

.. note::

   Prototype of the output that the ``.. testmodule::`` directive will generate.
   Test IDs (``TSPEC-QUEUE-*``) and requirement references come from ``@testid`` and
   ``@reqref`` Doxygen custom commands in the C source; the extension extracts them from
   the Doxygen XML, rendering requirement links natively without the ``@verbatim embed:rst``
   workaround currently used with Breathe.

Test Scenarios
==============

The following scenarios are defined in ``testcase.yaml``.  Each scenario
is a distinct build and execution configuration applied to all suites in
this module.

.. list-table::
   :header-rows: 1
   :widths: 37 25 38

   * - Scenario
     - Tags
     - Extra Configuration
   * - ``kernel.queue``
     - ``kernel``, ``userspace``
     - *(default — no extra Kconfig)*
   * - ``kernel.queue.minimallibc``
     - ``kernel``, ``userspace``, ``libc``
     - ``CONFIG_MINIMAL_LIBC=y``
       *(filter:* ``CONFIG_MINIMAL_LIBC_SUPPORTED`` *)*

Queue API Suite (``queue_api``)
================================

*k_queue API tests covering ISR/thread contexts, memory allocation,
multi-queue independence, user-space permissions, and unique-append.*

Contains tests that do not require single-CPU pinning.  Covers
ISR-to-thread and thread-to-ISR data passing via all five insertion
APIs, implicit memory allocation with heap-exhaustion and success
verification, simultaneous operation of multiple independent queues,
user-mode permission enforcement (expected kernel oops on unpermissioned
``k_queue`` access), and rejection of duplicate items by
``k_queue_unique_append()``.

Test Cases
----------

.. _testspec-queue_api-access_kernel_obj_with_priv_data:

Test access kernel obj with priv data (test_access_kernel_obj_with_priv_data)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   A user-mode thread accessing a ``k_queue`` object without permission
   triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-001
   :See: Here appear the links to the functions under test as per doxygen
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga9fa8884f88d5abdbbb63f12173e38df1>`_
   :Source: test_queue_contexts.c (line 792)

.. _testspec-queue_api-auto_free:

Test auto free (test_auto_free)
+++++++++++++++++++++++++++++++

   Implicitly-allocated queue elements and kernel objects from the
   preceding tests are automatically freed, leaving the test pool fully
   available.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-002
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-14`
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gab194fe3b40e86eb2c6178e372cc8d8c7>`_
   :Source: test_queue_user.c (line 319)

.. _testspec-queue_api-multiple_queues:

Test multiple queues (test_multiple_queues)
+++++++++++++++++++++++++++++++++++++++++++

   Multiple independent queues can be initialised and operated
   simultaneously without interference.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-003
   :Status: Draft
   :See: ``k_queue_init()``, ``tqueue_append()``, ``tqueue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga250b5cad2ec049439a5568b979bbb9aa>`_
   :Source: test_queue_contexts.c (line 723)

.. _testspec-queue_api-queue_alloc:

Test queue alloc (test_queue_alloc)
++++++++++++++++++++++++++++++++++++

   ``k_queue_alloc_append()`` / ``k_queue_alloc_prepend()`` fail
   gracefully without a resource pool and succeed when a sufficient pool
   is assigned.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-004
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-14`
   :See: ``k_queue_init()``, ``k_queue_alloc_append()``, ``k_queue_alloc_prepend()``, ``k_queue_is_empty()``, ``k_queue_get()``, ``k_queue_remove()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gaed02bb7c545405e0dd51019037e7522b>`_
   :Source: test_queue_contexts.c (line 562)

.. _testspec-queue_api-queue_alloc_append_null:

Test queue alloc append null (test_queue_alloc_append_null)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   Calling ``k_queue_alloc_append()`` with a NULL queue pointer from
   user mode triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-005
   :Status: Draft
   :See: ``k_queue_alloc_append()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gaacbdb2a8ad3d0df49424dd45472c0ff2>`_
   :Source: test_queue_fail.c (line 280)

.. _testspec-queue_api-queue_alloc_append_user:

Test queue alloc append user (test_queue_alloc_append_user)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   ``k_queue_alloc_append()`` produces FIFO order from user mode: items
   appended in ascending data order are dequeued in the same order.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-006
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-3`, :external+req:ref:`zep-srs-20-6`, :external+req:ref:`zep-srs-20-14`
   :See: ``k_queue_init()``, ``k_queue_alloc_append()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gaa517a1497728197f4cada7f397ba6fee>`_
   :Source: test_queue_user.c (line 261)

.. _testspec-queue_api-queue_alloc_prepend_null:

Test queue alloc prepend null (test_queue_alloc_prepend_null)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   Calling ``k_queue_alloc_prepend()`` with a NULL queue pointer from
   user mode triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-007
   :Status: Draft
   :See: ``k_queue_alloc_prepend()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gaec1ce0a17ea48f08b35dd458a85d8cce>`_
   :Source: test_queue_fail.c (line 312)

.. _testspec-queue_api-queue_alloc_prepend_user:

Test queue alloc prepend user (test_queue_alloc_prepend_user)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   ``k_queue_alloc_prepend()`` produces LIFO order from user mode: items
   prepended in ascending data order are dequeued in descending order.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-008
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-4`, :external+req:ref:`zep-srs-20-6`, :external+req:ref:`zep-srs-20-14`
   :See: ``k_queue_init()``, ``k_queue_alloc_prepend()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gacf4a07403bef6fa6caa29c0abbfbf960>`_
   :Source: test_queue_user.c (line 204)

.. _testspec-queue_api-queue_append_list_error:

Test queue append list error (test_queue_append_list_error)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   ``k_queue_append_list()`` returns ``-EINVAL`` for a NULL head or
   tail, and correctly wakes a waiting thread when given a valid
   single-node list.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-009
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-11`
   :See: ``k_queue_init()``, ``k_queue_append_list()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga1b66e52c74e2329141e2ba65b0d82e7c>`_
   :Source: test_queue_fail.c (line 115)

.. _testspec-queue_api-queue_cancel_wait_error:

Test queue cancel wait error (test_queue_cancel_wait_error)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   ``k_queue_cancel_wait()`` is a no-op on a queue with no waiting
   thread; calling it with NULL from user mode triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-010
   :Status: Draft
   :See: ``k_queue_init()``, ``k_queue_cancel_wait()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gac1dc28c3ee4c6096c06fcc63c0b36f5a>`_
   :Source: test_queue_fail.c (line 447)

.. _testspec-queue_api-queue_get_null:

Test queue get null (test_queue_get_null)
+++++++++++++++++++++++++++++++++++++++++

   Calling ``k_queue_get()`` with a NULL queue pointer from user mode
   triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-011
   :Status: Draft
   :See: ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga5de910e4136d8ab5b3156766581e9a97>`_
   :Source: test_queue_fail.c (line 344)

.. _testspec-queue_api-queue_init_null:

Test queue init null (test_queue_init_null)
+++++++++++++++++++++++++++++++++++++++++++

   Calling ``k_queue_init()`` with a NULL pointer from user mode
   triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-012
   :Status: Draft
   :See: ``k_queue_init()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga54cff7c6001a273568c1bf1470e1c873>`_
   :Source: test_queue_fail.c (line 255)

.. _testspec-queue_api-queue_is_empty_null:

Test queue is empty null (test_queue_is_empty_null)
+++++++++++++++++++++++++++++++++++++++++++++++++++

   Calling ``k_queue_is_empty()`` with a NULL queue pointer from user
   mode triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-013
   :Status: Draft
   :See: ``k_queue_is_empty()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga0ea444f9d37157c2a147423a54a5487e>`_
   :Source: test_queue_fail.c (line 369)

.. _testspec-queue_api-queue_isr2thread:

Test queue isr2thread (test_queue_isr2thread)
+++++++++++++++++++++++++++++++++++++++++++++

   Items enqueued from thread context are correctly delivered to the
   dequeuing ISR.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-014
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-6`
   :See: ``k_queue_init()``, ``k_queue_insert()``, ``k_queue_peek_tail()``, ``k_queue_append()``, ``k_queue_prepend()``, ``k_queue_append_list()``, ``k_queue_merge_slist()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#gaf33994f0f39f2b46dfaf2af17ccf99fb>`_
   :Source: test_queue_contexts.c (line 350)

.. _testspec-queue_api-queue_merge_list_error:

Test queue merge list error (test_queue_merge_list_error)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   ``k_queue_merge_slist()`` returns ``-EINVAL`` for an empty slist or a
   slist with NULL tail.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-015
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-12`
   :See: ``k_queue_init()``, ``k_queue_merge_slist()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga9b3e57bae9d525826336df40eb898427>`_
   :Source: test_queue_fail.c (line 202)

.. _testspec-queue_api-queue_peek_head_null:

Test queue peek head null (test_queue_peek_head_null)
+++++++++++++++++++++++++++++++++++++++++++++++++++++

   Calling ``k_queue_peek_head()`` with a NULL queue pointer from user
   mode triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-016
   :Status: Draft
   :See: ``k_queue_peek_head()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga0573db4d5d51480c6de8ac0b078cf939>`_
   :Source: test_queue_fail.c (line 394)

.. _testspec-queue_api-queue_peek_tail_null:

Test queue peek tail null (test_queue_peek_tail_null)
+++++++++++++++++++++++++++++++++++++++++++++++++++++

   Calling ``k_queue_peek_tail()`` with a NULL queue pointer from user
   mode triggers a kernel fault.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-017
   :Status: Draft
   :See: ``k_queue_peek_tail()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga6e44e5cff0edfd4a94ca8a73e590ab5b>`_
   :Source: test_queue_fail.c (line 419)

.. _testspec-queue_api-queue_thread2isr:

Test queue thread2isr (test_queue_thread2isr)
+++++++++++++++++++++++++++++++++++++++++++++

   Items enqueued from ISR context are correctly delivered to the
   dequeuing thread.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-018
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-6`
   :See: ``k_queue_init()``, ``k_queue_insert()``, ``k_queue_peek_tail()``, ``k_queue_append()``, ``k_queue_prepend()``, ``k_queue_append_list()``, ``k_queue_merge_slist()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga4e99828657b64f710660f7c49c79fe9f>`_
   :Source: test_queue_contexts.c (line 307)

.. _testspec-queue_api-queue_unique_append:

Test queue unique append (test_queue_unique_append)
+++++++++++++++++++++++++++++++++++++++++++++++++++

   ``k_queue_unique_append()`` rejects a duplicate item already present
   in the queue.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-API-019
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-13`
   :See: ``k_queue_init()``, ``k_queue_unique_append()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api.html#ga459dda9caedbf5d1adf693f7fcc22be7>`_
   :Source: test_queue_contexts.c (line 1026)

Queue API 1CPU Suite (``queue_api_1cpu``)
==========================================

*k_queue API tests requiring single-CPU pinning — thread-to-thread
passing, concurrent consumer dispatch, and priority-ordered delivery.*

Contains tests pinned to a single CPU via
``ztest_simple_1cpu_before`` / ``ztest_simple_1cpu_after``.  Covers
thread-to-thread data passing for both runtime-initialised and
compile-time-defined queues, correct item delivery to each of two
concurrent waiting threads, absence of the historical
``CONFIG_POLL`` race condition, and priority-then-FIFO dispatch ordering
when three threads at two priority levels compete for items.

Test Cases
----------

.. _testspec-queue_api_1cpu-queue_get_2threads:

Test queue get 2threads (test_queue_get_2threads)
+++++++++++++++++++++++++++++++++++++++++++++++++

   Each of two threads waiting on a queue receives exactly one item when
   two items are appended.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-1CPU-001
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-6`
   :See: ``k_queue_init()``, ``k_queue_append()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api__1cpu.html#ga625832221d8e19245df0f01989cf680e>`_
   :Source: test_queue_contexts.c (line 456)

.. _testspec-queue_api_1cpu-queue_get_fail:

Test queue get fail (test_queue_get_fail)
+++++++++++++++++++++++++++++++++++++++++

   ``k_queue_get()`` returns NULL on an empty queue with both
   ``K_NO_WAIT`` and a finite timeout.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-1CPU-002
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-6`
   :See: ``k_queue_init()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api__1cpu.html#ga85974775e88b2531ec0523f5608f31e3>`_
   :Source: test_queue_fail.c (line 44)

.. _testspec-queue_api_1cpu-queue_loop:

Test queue loop (test_queue_loop)
+++++++++++++++++++++++++++++++++

   Queue operations remain correct across 32 repeated multi-context
   cycles of enqueue, dequeue, and remove.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-1CPU-003
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-3`, :external+req:ref:`zep-srs-20-4`, :external+req:ref:`zep-srs-20-5`, :external+req:ref:`zep-srs-20-6`
   :See: ``k_queue_init()``, ``k_queue_append()``, ``k_queue_prepend()``, ``k_queue_get()``, ``k_queue_remove()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api__1cpu.html#gadbd9ee6195052890fd1b61f4a016a8e8>`_
   :Source: test_queue_loop.c (line 256)

.. _testspec-queue_api_1cpu-queue_multithread_competition:

Test queue multithread competition (test_queue_multithread_competition)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   Queue dispatches items to waiting threads in priority-then-FIFO
   order.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-1CPU-004
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-6`, :external+req:ref:`zep-srs-20-7`
   :See: ``k_queue_init()``, ``k_queue_is_empty()``, ``k_queue_append()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api__1cpu.html#ga0e791336265c0bac2f7d7a90fa92dbe3>`_
   :Source: test_queue_contexts.c (line 925)

.. _testspec-queue_api_1cpu-queue_poll_race:

Test queue poll race (test_queue_poll_race)
+++++++++++++++++++++++++++++++++++++++++++

   ``k_queue_get()`` does not spuriously return NULL when two waiting
   threads race for items appended to the queue.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-1CPU-005
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-6`
   :See: ``k_queue_init()``, ``k_queue_append()``, ``k_queue_get()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api__1cpu.html#ga226b3bdc41d00baf8091e23c1fc7f7fc>`_
   :Source: test_queue_contexts.c (line 649)

.. _testspec-queue_api_1cpu-queue_supv_to_user:

Test queue supv to user (test_queue_supv_to_user)
+++++++++++++++++++++++++++++++++++++++++++++++++

   Items enqueued by a supervisor thread are fully verified and drained
   by a user-mode child thread, including a canceled blocking wait.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-1CPU-006
   :Status: Draft
   :Requirements: :external+req:ref:`zep-srs-20-3`, :external+req:ref:`zep-srs-20-6`, :external+req:ref:`zep-srs-20-8`, :external+req:ref:`zep-srs-20-9`, :external+req:ref:`zep-srs-20-14`
   :See: ``k_queue_init()``, ``k_queue_append()``, ``k_queue_alloc_append()``, ``k_queue_is_empty()``, ``k_queue_peek_head()``, ``k_queue_peek_tail()``, ``k_queue_get()``, ``k_queue_cancel_wait()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api__1cpu.html#ga8dbdde7179df55dbcfff1cbebae0901b>`_
   :Source: test_queue_user.c (line 116)

.. _testspec-queue_api_1cpu-queue_thread2thread:

Test queue thread2thread (test_queue_thread2thread)
+++++++++++++++++++++++++++++++++++++++++++++++++++

   A queue correctly transfers items between threads regardless of
   whether it was initialised at runtime or at compile time.

   Here appears the additional content extracted from doxygen comments inside
   the function body

   :Test-Id: TSPEC-QUEUE-1CPU-007
   :Status: Draft
   :See: ``k_queue_init()``, ``tqueue_thread_thread()``
   :Doxygen: `Doxygen ↗ <http://localhost:8000/doxygen-zephyr-safety-testspec/html/group__queue__api__1cpu.html#gac2b4619b80e450b376058832ce1442af>`_
   :Source: test_queue_contexts.c (line 265)
