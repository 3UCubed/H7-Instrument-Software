/*
 * packet_queue.c
 *
 *  Created on: Oct 18, 2024
 *      Author: jaredmorrison
 */

#include "packet_queue.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

#define MAX_MESSAGES 16
#define MAX_MESSAGE_SIZE sizeof(Packet_t)

static StaticQueue_t queue_cb;
static Packet_t queue_data[MAX_MESSAGES];
static osMessageQueueId_t packet_queue;

osMessageQueueAttr_t queue_attribute = {
		.name = "packet_queue",
		.cb_mem = &queue_cb,
		.cb_size = sizeof(queue_cb),
		.mq_mem = queue_data,
		.mq_size = sizeof(queue_data),
};

void queue_init()
{
	packet_queue = osMessageQueueNew(MAX_MESSAGES, MAX_MESSAGE_SIZE, &queue_attribute);
}

void enqueue(Packet_t packet)
{
	osMessageQueuePut(packet_queue, &packet, 0U, 0U);
}

Packet_t dequeue()
{
	Packet_t dequeued_packet;

	if (osMessageQueueGetCount(packet_queue) == 0)
	{
		dequeued_packet.size = 0;
	}
	else
	{
		osMessageQueueGet(packet_queue, &dequeued_packet, 0U, 0U);
	}
	return dequeued_packet;
}