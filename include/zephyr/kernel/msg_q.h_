/*
 * Copyright (c) 2025, Tobias Kaestner, inovex GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Kernel Message Queue APIs
 */


 /**
 * @defgroup msgq_apis Message Queue APIs
 * @ingroup kernel_apis
 * @{
 */

/**
 * @brief Message Queue Structure
 */
struct k_msgq {
	/** Message queue wait queue */
	_wait_q_t wait_q;
	/** Lock */
	struct k_spinlock lock;
	/** Message size */
	size_t msg_size;
	/** Maximal number of messages */
	uint32_t max_msgs;
	/** Start of message buffer */
	char *buffer_start;
	/** End of message buffer */
	char *buffer_end;
	/** Read pointer */
	char *read_ptr;
	/** Write pointer */
	char *write_ptr;
	/** Number of used messages */
	uint32_t used_msgs;

	Z_DECL_POLL_EVENT

	/** Message queue flags*/
	uint8_t flags;

	SYS_PORT_TRACING_TRACKING_FIELD(k_msgq)

#ifdef CONFIG_OBJ_CORE_MSGQ
    /** Message queue object core representation, only when @kconfig{CONFIG_OBJ_CORE_MSGQ}*/
	struct k_obj_core  obj_core;
#endif
};
/**
 * @cond INTERNAL_HIDDEN
 */


#define Z_MSGQ_INITIALIZER(obj, q_buffer, q_msg_size, q_max_msgs) \
	{ \
	.wait_q = Z_WAIT_Q_INIT(&obj.wait_q), \
	.lock = {}, \
	.msg_size = q_msg_size, \
	.max_msgs = q_max_msgs, \
	.buffer_start = q_buffer, \
	.buffer_end = q_buffer + (q_max_msgs * q_msg_size), \
	.read_ptr = q_buffer, \
	.write_ptr = q_buffer, \
	.used_msgs = 0, \
	Z_POLL_EVENT_OBJ_INIT(obj) \
	.flags = 0, \
	}

#define K_MSGQ_FLAG_ALLOC	BIT(0)

/**
 * INTERNAL_HIDDEN @endcond
 */



/**
 * @brief Message Queue Attributes
 */
struct k_msgq_attrs {
	/** Message Size */
	size_t msg_size;
	/** Maximal number of messages */
	uint32_t max_msgs;
	/** Used messages */
	uint32_t used_msgs;
};


/**
 * @brief Statically define and initialize a message queue.
 *
 * The message queue's ring buffer contains space for @a q_max_msgs messages,
 * each of which is @a q_msg_size bytes long. Alignment of the message queue's
 * ring buffer is not necessary, setting @a q_align to 1 is sufficient.
 *
 * The message queue can be accessed outside the module where it is defined
 * using:
 *
 * @code extern struct k_msgq <name>; @endcode
 *
 * @param q_name Name of the message queue.
 * @param q_msg_size Message size (in bytes).
 * @param q_max_msgs Maximum number of messages that can be queued.
 * @param q_align Alignment of the message queue's ring buffer (power of 2).
 *
 */
#define K_MSGQ_DEFINE(q_name, q_msg_size, q_max_msgs, q_align)		\
	static char __noinit __aligned(q_align)				\
		_k_fifo_buf_##q_name[(q_max_msgs) * (q_msg_size)];	\
	STRUCT_SECTION_ITERABLE(k_msgq, q_name) =			\
	       Z_MSGQ_INITIALIZER(q_name, _k_fifo_buf_##q_name,	\
				  (q_msg_size), (q_max_msgs))

/**
 * @brief Initialize a message queue.
 *
 * This routine initializes a message queue object, prior to its first use.
 *
 * The message queue's ring buffer must contain space for @a max_msgs messages,
 * each of which is @a msg_size bytes long. Alignment of the message queue's
 * ring buffer is not necessary.
 *
 * @param msgq Address of the message queue.
 * @param buffer Pointer to ring buffer that holds queued messages.
 * @param msg_size Message size (in bytes).
 * @param max_msgs Maximum number of messages that can be queued.
 *
 * @required-by{20-2}
 */
 void k_msgq_init(struct k_msgq *msgq, char *buffer, size_t msg_size,
		 uint32_t max_msgs);

/**
 * @brief Initialize a message queue.
 *
 * This routine initializes a message queue object, prior to its first use,
 * allocating its internal ring buffer from the calling thread's resource
 * pool.
 *
 * Memory allocated for the ring buffer can be released by calling
 * k_msgq_cleanup(), or if userspace is enabled and the msgq object loses
 * all of its references.
 *
 * @param msgq Address of the message queue.
 * @param msg_size Message size (in bytes).
 * @param max_msgs Maximum number of messages that can be queued.
 *
 * @return 0 on success, -ENOMEM if there was insufficient memory in the
 *	thread's resource pool, or -EINVAL if the size parameters cause
 *	an integer overflow.
 */
__syscall int k_msgq_alloc_init(struct k_msgq *msgq, size_t msg_size,
				uint32_t max_msgs);

/**
 * @brief Release allocated buffer for a queue
 *
 * Releases memory allocated for the ring buffer.
 *
 * @param msgq message queue to cleanup
 *
 * @retval 0 on success
 * @retval -EBUSY Queue not empty
 */
int k_msgq_cleanup(struct k_msgq *msgq);

/**
 * @brief Send a message to the end of a message queue.
 *
 * This routine sends a message to message queue @c q.
 *
 * @note The message content is copied from @c data into @c msgq and the @c data
 * pointer is not retained, so the message content will not be modified
 * by this function.
 *
 * @par
 * 
 * @note 
 * Another note
 * 
 * @funcprops 
 *
 *  @isr_ok
 *  @reschedule
 * 
 * 
 * @param msgq Address of the message queue.
 * @param data Pointer to the message.
 * @param timeout Waiting period to add the message, or one of the special
 *                values K_NO_WAIT and K_FOREVER.
 *
 * @retval 0 Message sent.
 * @retval -ENOMSG Returned without waiting or queue purged.
 * @retval -EAGAIN Waiting period timed out.
 */
__syscall int k_msgq_put(struct k_msgq *msgq, const void *data, k_timeout_t timeout);

/**
 * @brief Send a message to the front of a message queue.
 *
 * This routine sends a message to the beginning (head) of message queue @a q.
 * Messages sent with this method will be retrieved before any pre-existing
 * messages in the queue.
 *
 * @note if there is no space in the message queue, this function will
 * behave the same as k_msgq_put.
 *
 * @note The message content is copied from @a data into @a msgq and the @a data
 * pointer is not retained, so the message content will not be modified
 * by this function.
 *
 * @funcprops \isr_ok
 *
 * @param msgq Address of the message queue.
 * @param data Pointer to the message.
 * @param timeout Waiting period to add the message, or one of the special
 *                values K_NO_WAIT and K_FOREVER.
 *
 * @retval 0 Message sent.
 * @retval -ENOMSG Returned without waiting or queue purged.
 * @retval -EAGAIN Waiting period timed out.
 */
__syscall int k_msgq_put_front(struct k_msgq *msgq, const void *data, k_timeout_t timeout);

/**
 * @brief Receive a message from a message queue.
 *
 * This routine receives a message from message queue @a q in a "first in,
 * first out" manner.
 *
 * @note @a timeout must be set to K_NO_WAIT if called from ISR.
 *
 * @funcprops \isr_ok
 *
 * @param msgq Address of the message queue.
 * @param data Address of area to hold the received message.
 * @param timeout Waiting period to receive the message,
 *                or one of the special values K_NO_WAIT and
 *                K_FOREVER.
 *
 * @retval 0 Message received.
 * @retval -ENOMSG Returned without waiting or queue purged.
 * @retval -EAGAIN Waiting period timed out.
 */
__syscall int k_msgq_get(struct k_msgq *msgq, void *data, k_timeout_t timeout);

/**
 * @brief Peek/read a message from a message queue.
 *
 * This routine reads a message from message queue @a q in a "first in,
 * first out" manner and leaves the message in the queue.
 *
 * @funcprops \isr_ok
 *
 * @param msgq Address of the message queue.
 * @param data Address of area to hold the message read from the queue.
 *
 * @retval 0 Message read.
 * @retval -ENOMSG Returned when the queue has no message.
 */
__syscall int k_msgq_peek(struct k_msgq *msgq, void *data);

/**
 * @brief Peek/read a message from a message queue at the specified index
 *
 * This routine reads a message from message queue at the specified index
 * and leaves the message in the queue.
 * k_msgq_peek_at(msgq, data, 0) is equivalent to k_msgq_peek(msgq, data)
 *
 * @funcprops \isr_ok
 *
 * @param msgq Address of the message queue.
 * @param data Address of area to hold the message read from the queue.
 * @param idx Message queue index at which to peek
 *
 * @retval 0 Message read.
 * @retval -ENOMSG Returned when the queue has no message at index.
 */
__syscall int k_msgq_peek_at(struct k_msgq *msgq, void *data, uint32_t idx);

/**
 * @brief Purge a message queue.
 *
 * This routine discards all unreceived messages in a message queue's ring
 * buffer. Any threads that are blocked waiting to send a message to the
 * message queue are unblocked and see an -ENOMSG error code.
 *
 * @param msgq Address of the message queue.
 */
__syscall void k_msgq_purge(struct k_msgq *msgq);

/**
 * @brief Get the amount of free space in a message queue.
 *
 * This routine returns the number of unused entries in a message queue's
 * ring buffer.
 *
 * @param msgq Address of the message queue.
 *
 * @return Number of unused ring buffer entries.
 */
__syscall uint32_t k_msgq_num_free_get(struct k_msgq *msgq);

/**
 * @brief Get basic attributes of a message queue.
 *
 * This routine fetches basic attributes of message queue into attr argument.
 *
 * @param msgq Address of the message queue.
 * @param attrs pointer to message queue attribute structure.
 */
__syscall void  k_msgq_get_attrs(struct k_msgq *msgq,
				 struct k_msgq_attrs *attrs);


static inline uint32_t z_impl_k_msgq_num_free_get(struct k_msgq *msgq)
{
	return msgq->max_msgs - msgq->used_msgs;
}

/**
 * @brief Get the number of messages in a message queue.
 *
 * This routine returns the number of messages in a message queue's ring buffer.
 *
 * @param msgq Address of the message queue.
 *
 * @return Number of messages.
 */
__syscall uint32_t k_msgq_num_used_get(struct k_msgq *msgq);

static inline uint32_t z_impl_k_msgq_num_used_get(struct k_msgq *msgq)
{
	return msgq->used_msgs;
}

/** @} */
