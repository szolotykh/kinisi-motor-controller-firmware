//------------------------------------------------------------
// File name: message_queue.c
// Description: Assemble fragmented length-prefixed frames into a bounded receive queue.
//------------------------------------------------------------
#include "message_queue.h"
#include <string.h>

// Initialize the message queue
/**
 * @brief Reset all queue positions and discard an incomplete frame.
 * @note Caller must exclude concurrent producers/consumers during reset.
 */
void init_queue(message_queue_t *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->incomplete_count = 0;
    queue->incomplete_barriers = 0;
}

void mark_pending_frames(message_queue_t *queue, uint16_t barriers) {
    for (unsigned i = 0, slot = queue->head; i < queue->count; ++i) {
        queue->wake_barriers[slot] |= barriers;
        slot = (slot + 1) % MESSAGE_QUEUE_MAX_SIZE;
    }
    if (queue->incomplete_count) queue->incomplete_barriers |= barriers;
}

// Check if the queue is empty
/**
 * @brief Check whether the queue contains no complete frames.
 */
int is_queue_empty(message_queue_t *queue) {
    return queue->count == 0;
}

// Check if the queue is full
/**
 * @brief Check whether all complete-frame slots are occupied.
 */
int is_queue_full(message_queue_t *queue) {
    return queue->count == MESSAGE_QUEUE_MAX_SIZE;
}

// Enqueue a single message into the queue
/**
 * @brief Copy one complete length-prefixed frame into the queue.
 * @return Nonzero if accepted, or zero when full.
 * @note Caller supplies all declared bytes and serializes queue access.
 */
int enqueue(message_queue_t *queue, char *message) {
    unsigned int length = (unsigned char)message[0];
    if (is_queue_full(queue)) {
        return 0;
    }
    memcpy(queue->messages[queue->tail], message, length + 1);
    queue->wake_barriers[queue->tail] = 0;
    queue->tail = (queue->tail + 1) % MESSAGE_QUEUE_MAX_SIZE;
    queue->count++;
    return 1;
}

// Enqueue multiple messages into the queue
/**
 * @brief Assemble a byte stream and enqueue each completed length-prefixed frame.
 * @return Nonzero if all completed frames fit; zero if any were dropped.
 * @note Partial frames are retained across calls; caller serializes queue access.
 */
int enqueue_multi(message_queue_t *queue, char *messages, unsigned int length) {
    int accepted = 1;
    // Assemble one complete length-prefixed frame at a time. The one-byte
    // length bounds every frame to 256 bytes, including malformed commands.
    for (unsigned int offset = 0; offset < length; ++offset) {
        queue->incomplete[queue->incomplete_count++] = messages[offset];
        const unsigned int frame_length = (unsigned char)queue->incomplete[0] + 1U;
        if (queue->incomplete_count == frame_length) {
            unsigned slot = queue->tail;
            if (!enqueue(queue, queue->incomplete)) accepted = 0;
            else queue->wake_barriers[slot] = queue->incomplete_barriers;
            queue->incomplete_barriers = 0;
            queue->incomplete_count = 0;
        }
    }
    return accepted;
}

// Dequeue a single message from the queue
/**
 * @brief Remove one queued frame and copy its bytes without the length prefix.
 * @param message Destination large enough for a maximum-size frame.
 * @param message_len Receives the payload length, or zero if the queue is empty.
 * @note Caller serializes access and can check emptiness to distinguish empty frames.
 */
int dequeue(message_queue_t *queue, char *message, int *message_len) {
    if (is_queue_empty(queue)) {
        *message_len = 0;
        return 0;
    }
    char *msg = queue->messages[queue->head];
    unsigned char length = msg[0];
    memcpy(message, msg + 1, length);
    *message_len = length;
    uint16_t barriers = queue->wake_barriers[queue->head];
    queue->head = (queue->head + 1) % MESSAGE_QUEUE_MAX_SIZE;
    queue->count--;
    return barriers;
}
