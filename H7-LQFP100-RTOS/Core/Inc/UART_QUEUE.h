/*
 * UART_QUEUE.h
 *
 *  Created on: May 21, 2024
 *      Author: jaredmorrison
 */

#ifndef INC_UART_QUEUE_H_
#define INC_UART_QUEUE_H_


#include <stdint.h>
#include <stdbool.h>

// Define the packet structure
typedef struct {
    uint8_t* array;  // Pointer to the array data
    uint16_t size;   // Size of the array
} packet_t;

// Define the queue structure
typedef struct {
    packet_t* packets;   // Array of packets
    uint16_t max_size;   // Maximum number of packets in the queue
    uint16_t head;       // Index of the head of the queue
    uint16_t tail;       // Index of the tail of the queue
    uint16_t count;      // Number of packets currently in the queue
} uart_queue_t;

// Function prototypes
void queue_init(uart_queue_t* queue, uint16_t max_size);
void queue_free(uart_queue_t* queue);
bool queue_enqueue(uart_queue_t* queue, const packet_t* packet);
bool queue_dequeue(uart_queue_t* queue, packet_t* packet);
bool queue_is_empty(const uart_queue_t* queue);
bool queue_is_full(const uart_queue_t* queue);



#endif /* INC_UART_QUEUE_H_ */
