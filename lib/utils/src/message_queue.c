#include "message_queue.h"
#include <string.h>

// Initialize the message queue
void init_queue(message_queue_t *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->incomplete_count = 0;
}

// Check if the queue is empty
int is_queue_empty(message_queue_t *queue) {
    return queue->count == 0;
}

// Check if the queue is full
int is_queue_full(message_queue_t *queue) {
    return queue->count == MESSAGE_QUEUE_MAX_SIZE;
}

// Enqueue a single message into the queue
int enqueue(message_queue_t *queue, char *message) {
    unsigned int length = message[0];
    if (is_queue_full(queue)) {
        return 0;
    }
    memcpy(queue->messages[queue->tail], message, length + 1);
    queue->tail = (queue->tail + 1) % MESSAGE_QUEUE_MAX_SIZE;
    queue->count++;
    return 1;
}

// Enqueue multiple messages into the queue
int enqueue_multi(message_queue_t *queue, char *messages, unsigned int length) {
    unsigned int offset = 0;
    unsigned int remaining = length;
    unsigned int i = 0;

    // Handle incomplete messages from the previous call
    if (queue->incomplete_count > 0) {
        memcpy(queue->incomplete + queue->incomplete_count, messages, MESSAGE_QUEUE_MAX_STR_LENGTH - queue->incomplete_count);
        remaining -= (MESSAGE_QUEUE_MAX_STR_LENGTH - queue->incomplete_count);
        offset += (MESSAGE_QUEUE_MAX_STR_LENGTH - queue->incomplete_count);
        if (remaining >= queue->incomplete[0]) {
            memcpy(queue->messages[queue->tail], queue->incomplete, queue->incomplete[0] + 1);
            queue->tail = (queue->tail + 1) % MESSAGE_QUEUE_MAX_SIZE;
            queue->count++;
            queue->incomplete_count = 0;
        } else {
            memcpy(queue->incomplete, messages + offset, remaining);
            queue->incomplete_count = remaining;
            return 1;
        }
    }

    // Enqueue complete messages
    while (offset < length) {
        unsigned int message_length = messages[offset];
        if (offset + message_length + 1 <= length) {
            if (!enqueue(queue, messages + offset)) {
                return 0;
            }
            offset += message_length + 1;
        } else {
            memcpy(queue->incomplete, messages + offset, length - offset);
            queue->incomplete_count = length - offset;
            return 1;
        }
    }
    return 1;
}

// Dequeue a single message from the queue
void dequeue(message_queue_t *queue, char *message, int *message_len) {
    if (is_queue_empty(queue)) {
        *message_len = 0;
        return;
    }
    char *msg = queue->messages[queue->head];
    unsigned char length = msg[0];
    memcpy(message, msg + 1, length);
    *message_len = length;
    queue->head = (queue->head + 1) % MESSAGE_QUEUE_MAX_SIZE;
    queue->count--;
}
