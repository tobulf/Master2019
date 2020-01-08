/* 
* EventQueue.h
*
* Created: 08.01.2020 14:42:03
* Author: Tobias
*/


#ifndef __EVENTQUEUE_H__
#define __EVENTQUEUE_H__
#include <stdbool.h>
#include "drivers/Debug.h"

#define EVENT_LENGTH 44
#define EVENT_DATA_LENGTH 20
#define RING_BUF_LENGTH 220

class EventQueue{
//functions
public:
	void push_event(uint32_t timestamp, int16_t* event_data);
	uint8_t* pop_event(void);
	bool queue_full(void);
	bool queue_empty(void);
	uint8_t length(void);
	EventQueue();
protected:
private:
	// 44 bytes, for each event, 440 bytes = 10 events lasting 1s each.
	uint8_t ring_buf[RING_BUF_LENGTH];
	uint8_t num_events;
	uint16_t head;
	uint16_t tail;
	bool full_buf;
	bool empty_buf;
	void check_if_queue_empty(void);
	void check_if_queue_full(void);
}; //EventQueue

#endif //__EVENTQUEUE_H__
