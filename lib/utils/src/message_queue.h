//------------------------------------------------------------
// File name: message_queue.h
// Description: Declare bounded receive queues; callers serialize IRQ and task access.
//------------------------------------------------------------
#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <string.h>

// Maximum number of messages that can be stored in the queue
#define MESSAGE_QUEUE_MAX_SIZE 30

// Maximum size of a message in bytes (including length byte)
#define MESSAGE_QUEUE_MAX_STR_LENGTH 256

// Structure to hold the message queue
typedef struct {
    char messages[MESSAGE_QUEUE_MAX_SIZE][MESSAGE_QUEUE_MAX_STR_LENGTH]; // 2D array to hold messages
    unsigned int head; // Index of the first message in the queue
    unsigned int tail; // Index of the next available slot in the queue
    unsigned int count; // Number of messages in the queue
    char incomplete[MESSAGE_QUEUE_MAX_STR_LENGTH]; // Buffer to hold incomplete messages
    unsigned int incomplete_count; // Number of bytes in the incomplete message buffer
} message_queue_t;

// Initialize the message queue
/**
 * @brief Reset all queue positions and discard an incomplete frame.
 * @note Caller must exclude concurrent producers/consumers during reset.
 */
void init_queue(message_queue_t *queue);

// Check if the queue is empty
/**
 * @brief Check whether the queue contains no complete frames.
 */
int is_queue_empty(message_queue_t *queue);

// Check if the queue is full
/**
 * @brief Check whether all complete-frame slots are occupied.
 */
int is_queue_full(message_queue_t *queue);

// Enqueue a single message into the queue
/**
 * @brief Copy one complete length-prefixed frame into the queue.
 * @return Nonzero if accepted, or zero when full.
 * @note Caller supplies all declared bytes and serializes queue access.
 */
int enqueue(message_queue_t *queue, char *message);

// Enqueue multiple messages into the queue
/**
 * @brief Assemble a byte stream and enqueue each completed length-prefixed frame.
 * @return Nonzero if all completed frames fit; zero if any were dropped.
 * @note Partial frames are retained across calls; caller serializes queue access.
 */
int enqueue_multi(message_queue_t *queue, char *messages, unsigned int length);

// Dequeue a single message from the queue
/**
 * @brief Remove one queued frame and copy its bytes without the length prefix.
 * @param message Destination large enough for a maximum-size frame.
 * @param message_len Receives the payload length, or zero if the queue is empty.
 * @note Caller serializes access and can check emptiness to distinguish empty frames.
 */
void dequeue(message_queue_t *queue, char *message, int *message_len);

#endif
