#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <string.h>

// Maximum number of messages that can be stored in the queue
#define MESSAGE_QUEUE_MAX_SIZE 30

// Maximum size of a message in bytes (including length byte)
#define MESSAGE_QUEUE_MAX_STR_LENGTH 50

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
void init_queue(message_queue_t *queue);

// Check if the queue is empty
int is_queue_empty(message_queue_t *queue);

// Check if the queue is full
int is_queue_full(message_queue_t *queue);

// Enqueue a single message into the queue
int enqueue(message_queue_t *queue, char *message);

// Enqueue multiple messages into the queue
int enqueue_multi(message_queue_t *queue, char *messages, unsigned int length);

// Dequeue a single message from the queue
void dequeue(message_queue_t *queue, char *message, int *message_len);

#endif
