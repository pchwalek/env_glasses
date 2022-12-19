/*
 * circular_buffer.c
 *
 *  Created on: Dec 4, 2019
 *      Author: giand
 */
#include "circular_buffer.h"

uint8_t empty(CircularBuffer *self) {
	return self->read_idx_ == self->write_idx_;
}

uint32_t size(CircularBuffer *self) {
	return self->write_idx_ - self->read_idx_;
}

uint32_t front(CircularBuffer *self) {
	return self->elems_[self->read_idx_ & MAX_SIZE];
}

uint32_t pop_front(CircularBuffer *self) {
	if (!empty(self)) {
		//free(self->elems_[self->read_idx_ & MAX_SIZE]);
		uint32_t element = self->elems_[self->read_idx_ & MAX_SIZE];
		self->elems_[self->read_idx_ & MAX_SIZE] = NULL;
		self->read_idx_ += 1;
		return element;
	} else {
		return 0;
	}
}

uint32_t push_back(CircularBuffer *self) {
//	self->elems_[self->write_idx_ & MAX_SIZE] = elem;
	uint32_t addr = self->elems_[self->write_idx_ & MAX_SIZE];

	if( (self->write_idx_ - self->read_idx_) == MAX_SIZE){
		self->read_idx_++; //overwrite condition should increment read_idx_
	}
	self->write_idx_++;

	return addr;
}

CircularBuffer* create_circular_buffer(uint32_t n, uint32_t start_addr, uint32_t packetSize) {
   CircularBuffer* s = (CircularBuffer*) malloc(sizeof(CircularBuffer)+n*sizeof(uint32_t));
   if (!s) return NULL;
   s->bufSize = n;
   s->read_idx_ = 0;
   s->write_idx_ = 0;
   for (uint32_t i=0; i<n; i++) {
	   s->elems_[i] = start_addr;
	   start_addr += packetSize;
   }
   return s;
}
