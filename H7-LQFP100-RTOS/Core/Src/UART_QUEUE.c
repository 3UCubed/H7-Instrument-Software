/*
 * UART_QUEUE.c
 *
 *  Created on: May 21, 2024
 *      Author: jaredmorrison
 */


#include "UART_QUEUE.h"
#include <stdlib.h>

// Initialize the queue
void queue_init(uart_queue_t* queue, uint16_t max_size) {
    queue->packets = (packet_t*)malloc(max_size * sizeof(packet_t));
    queue->max_size = max_size;
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

// Free the queue memory
void queue_free(uart_queue_t* queue) {
    free(queue->packets);
}

// Enqueue a packet
bool queue_enqueue(uart_queue_t* queue, const packet_t* packet) {
    if (queue->count >= queue->max_size) {
        // Queue is full
        return false;
    }
    queue->packets[queue->tail] = *packet;
    queue->tail = (queue->tail + 1) % queue->max_size;
    queue->count++;
    return true;
}

// Dequeue a packet
bool queue_dequeue(uart_queue_t* queue, packet_t* packet) {
    if (queue->count == 0) {
        // Queue is empty
        return false;
    }
    *packet = queue->packets[queue->head];
    queue->head = (queue->head + 1) % queue->max_size;
    queue->count--;
    free(packet->array);

    return true;
}

// Check if the queue is empty
bool queue_is_empty(const uart_queue_t* queue) {
    return queue->count == 0;
}

// Check if the queue is full
bool queue_is_full(const uart_queue_t* queue) {
    return queue->count >= queue->max_size;
}
