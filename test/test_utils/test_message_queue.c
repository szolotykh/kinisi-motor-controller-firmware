#include "unity.h"
#include <message_queue.h>

// Declare a message queue for testing
message_queue_t test_queue;

void setUp(void) {
    // Initialize the test queue before each test
    init_queue(&test_queue);
}

void tearDown(void) {
    // No teardown required for this test suite
}

void test_init_queue(void) {
    // Test that the message queue is initialized properly
    TEST_ASSERT_EQUAL(0, test_queue.head);
    TEST_ASSERT_EQUAL(0, test_queue.tail);
    TEST_ASSERT_EQUAL(0, test_queue.count);
    TEST_ASSERT_EQUAL(0, test_queue.incomplete_count);
}

void test_is_queue_empty(void) {
    // Test that the queue is initially empty
    TEST_ASSERT_EQUAL(1, is_queue_empty(&test_queue));

    // Add a message to the queue and test again
    char message1[] = {4, 't', 'e', 's', 't'};
    TEST_ASSERT_EQUAL(1, enqueue(&test_queue, message1));
    TEST_ASSERT_EQUAL(0, is_queue_empty(&test_queue));
}

void test_is_queue_full(void) {
    // Test that the queue is initially not full
    TEST_ASSERT_EQUAL(0, is_queue_full(&test_queue));

    // Add 10 messages to the queue and test again
    char message[] = {4, 't', 'e', 's', 't'};
    for (int i = 0; i < MESSAGE_QUEUE_MAX_SIZE; i++) {
        TEST_ASSERT_EQUAL(1, enqueue(&test_queue, message));
    }
    TEST_ASSERT_EQUAL(1, is_queue_full(&test_queue));
}

void test_enqueue_single(void) {
    // Test that a single message can be enqueued
    char message[] = {4, 't', 'e', 's', 't'};
    TEST_ASSERT_EQUAL(1, enqueue(&test_queue, message));
    TEST_ASSERT_EQUAL(1, test_queue.count);

    // Test that the message can be dequeued correctly
    char dequeued_message[MESSAGE_QUEUE_MAX_STR_LENGTH];
    int message_len;
    dequeue(&test_queue, dequeued_message, &message_len);
    TEST_ASSERT_EQUAL_MEMORY(message + 1, dequeued_message, message[0]);
    TEST_ASSERT_EQUAL(4, message_len);
}

void test_enqueue_multi(void) {
    // Test that multiple messages can be enqueued
    char messages[] = {
        4, 't', 'e', 's', 't',
        7, 'm', 'e', 's', 's', 'a', 'g', 'e',
        3, 'h', 'i', '!'
    };
    TEST_ASSERT_EQUAL(1, enqueue_multi(&test_queue, messages, 14));
    TEST_ASSERT_EQUAL(3, test_queue.count);

    // Test that the messages can be dequeued correctly
    char dequeued_message1[MESSAGE_QUEUE_MAX_STR_LENGTH];
    char dequeued_message2[MESSAGE_QUEUE_MAX_STR_LENGTH];
    char dequeued_message3[MESSAGE_QUEUE_MAX_STR_LENGTH];
    int message_len;
    dequeue(&test_queue, dequeued_message1, &message_len);
    TEST_ASSERT_EQUAL_MEMORY(messages + 1, dequeued_message1, messages[0]);
    TEST_ASSERT_EQUAL(4, message_len);
    dequeue(&test_queue, dequeued_message2, &message_len);
    TEST_ASSERT_EQUAL_MEMORY(messages + 5, dequeued_message2, messages[4]);
    TEST_ASSERT_EQUAL(7, message_len);
    dequeue(&test_queue, dequeued_message3, &message_len);
    TEST_ASSERT_EQUAL_MEMORY(messages + 12, dequeued_message3, messages[11]);
    TEST_ASSERT_EQUAL(3, message_len);
}

void test_enqueue_multi_with_incomplete_message(void) {
    // Test that an incomplete message can be enqueued
    char messages[] = {
        4, 't', 'e', 's', 't',
        7, 'm', 'e', 's', 's', 'a', 'g', 'e',
        3, 'h', 'i', '!'
    };
    TEST_ASSERT_EQUAL(0, test_queue.incomplete_count); // Ensure incomplete buffer is empty
    TEST_ASSERT_EQUAL(1, enqueue_multi(&test_queue, messages, 11));
    TEST_ASSERT_EQUAL(1, test_queue.count);
    TEST_ASSERT_EQUAL(6, test_queue.incomplete_count);

    // Test that the remaining bytes are added to the incomplete buffer
    TEST_ASSERT_EQUAL(1, enqueue_multi(&test_queue, messages + 11, 2));
    TEST_ASSERT_EQUAL(2, test_queue.count);
    TEST_ASSERT_EQUAL(0, test_queue.incomplete_count);

    // Test that the complete message can be dequeued
    char dequeued_message1[MESSAGE_QUEUE_MAX_STR_LENGTH];
    char dequeued_message2[MESSAGE_QUEUE_MAX_STR_LENGTH];

    int message_len;
    dequeue(&test_queue, dequeued_message1, &message_len);
    TEST_ASSERT_EQUAL_MEMORY(messages + 1, dequeued_message1, messages[0]);
    TEST_ASSERT_EQUAL(4, message_len);

    dequeue(&test_queue, dequeued_message2, &message_len);
    TEST_ASSERT_EQUAL_MEMORY(messages + 6, dequeued_message2, messages[5]);
    TEST_ASSERT_EQUAL(7, message_len);
}

void test_dequeue_with_incomplete_message(void) {
    // Test dequeuing a message that is longer than the buffer
    char message[] = {4, 't', 'e', 's', 't'};
    TEST_ASSERT_EQUAL(1, enqueue(&test_queue, message));
    char messages[] = {7, 'm', 'e', 's', 's', 'a', 'g', 'e'};
    TEST_ASSERT_EQUAL(1, enqueue(&test_queue, messages));
    char incomplete[] = {3, 'h', 'i', '!'};
    memcpy(test_queue.incomplete, incomplete, 3);
    test_queue.incomplete_count = 3;
    test_queue.head = 1;
    test_queue.count = 2;

    // Test that the first message is skipped over and the incomplete message is used
    char dequeued_message[MESSAGE_QUEUE_MAX_STR_LENGTH];
    int message_len;
    dequeue(&test_queue, dequeued_message, &message_len);
    TEST_ASSERT_EQUAL_MEMORY(messages + 1, dequeued_message, messages[0]);
    TEST_ASSERT_EQUAL(7, message_len);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_queue);
    RUN_TEST(test_is_queue_empty);
    RUN_TEST(test_is_queue_full);
    RUN_TEST(test_enqueue_multi_with_incomplete_message);
    RUN_TEST(test_dequeue_with_incomplete_message);
    return UNITY_END();
}