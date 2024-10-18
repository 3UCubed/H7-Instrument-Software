/*
 * packet_queue.h
 *
 *  Created on: Oct 18, 2024
 *      Author: jaredmorrison
 */

#ifndef INC_PACKET_QUEUE_H_
#define INC_PACKET_QUEUE_H_

#include "packet_creation.h"


void queue_init();
void enqueue(Packet_t packet);
Packet_t dequeue();

#endif /* INC_PACKET_QUEUE_H_ */
