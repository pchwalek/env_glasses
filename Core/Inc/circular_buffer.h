#pragma once
#include "stm32wbxx_hal.h"
#include <stdlib.h>
#define MAX_SIZE 255

//typedef struct CircularBuffer {
//	void *elems_[256];
//	uint32_t read_idx_;
//	uint32_t write_idx_;
//} CircularBuffer;

typedef struct CircularBuffer {
	uint32_t bufSize;
	uint32_t read_idx_;
	uint32_t write_idx_;
	uint32_t elems_[];
} CircularBuffer;

uint8_t empty(CircularBuffer *self);
uint32_t size(CircularBuffer *self);
uint32_t front(CircularBuffer *self);
uint32_t pop_front(CircularBuffer *self);
//void append_back(CircularBuffer *self, uint32_t elem);
uint32_t push_back(CircularBuffer *self);

CircularBuffer* create_circular_buffer(uint32_t n, uint32_t start_addr, uint32_t packetSize);
