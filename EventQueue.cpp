/* 
* EventQueue.cpp
*
* Created: 08.01.2020 14:42:03
* Author: Tobias
*/


#include "EventQueue.h"


// default constructor
EventQueue::EventQueue(){
	head = 0;
	tail = 0;
	empty_buf = true;
	full_buf = false; 
} //EventQueue

void EventQueue::push_event(uint32_t timestamp, int16_t* event_data){
	//Store timestamp
	if (!full_buf){
		for(uint8_t i = 0; i<4 ; i++){
			ring_buf[head] = (uint8_t)((timestamp>>(8*(3-i))) & 0xFF);
			if (head != RING_BUF_LENGTH-1){
				head++;
			}
			else{
				head = 0;
			}
		}
		for (uint8_t i = 0; i < EVENT_DATA_LENGTH; i++){
			ring_buf[head]=(uint8_t)((event_data[i]>>8) & 0xFF);
			if (head != RING_BUF_LENGTH-1){
				head++;
			}
			else{
				head = 0;
			}
			ring_buf[head]=(uint8_t)(event_data[i] & 0xFF);
			if (head != RING_BUF_LENGTH-1){
				head++;
			}
			else{
				head = 0;
				}
		}
		empty_buf = false;
		num_events++;
		check_if_queue_full();
	}
}

uint8_t* EventQueue::pop_event(){
	if (!empty_buf){
		if (full_buf){full_buf = false;}
		num_events--;
		uint8_t old_tail = tail;
		tail = tail + EVENT_LENGTH;
		if (tail == RING_BUF_LENGTH){
			tail = 0;
			check_if_queue_empty();
			return &ring_buf[old_tail];	
		}
		else{
			check_if_queue_empty();
			return &ring_buf[old_tail];
		}
		
	}
	else{
		return NULL;
	}
}

void EventQueue::check_if_queue_full(){
	// When head and tail meet:
	if (tail == head){
		full_buf = true;
	}
	else{
		full_buf = false;
	}
}

void EventQueue::check_if_queue_empty(){
	if(head == tail){
		empty_buf = true;
	}
	else {
		empty_buf = false;
	}
}

bool EventQueue::queue_full(){
	return full_buf;
}

bool EventQueue::queue_empty(){
	return empty_buf;
}

uint8_t EventQueue::length(){
	return num_events;
}