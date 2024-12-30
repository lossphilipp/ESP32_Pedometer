#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <malloc.h>
#include <inttypes.h>
#include <stdbool.h>
#include "esp_log.h"

#define RINGBUFFER_SUCCESS               0
#define RINGBUFFER_ERROR_EMPTY          -1
#define RINGBUFFER_ERROR_FULL           -2
#define RINGBUFFER_ERROR_OUTOFMEMORY    -3
#define RINGBUFFER_TAG                  "RINGBUFFER"

typedef int32_t RingbufferHandle;

RingbufferHandle ringbuffer_create(uint32_t size, size_t element_size);
void ringbuffer_destroy(RingbufferHandle* pRingBufferHandle);
void ringbuffer_clear(RingbufferHandle ringbufferHandle);
bool ringbuffer_isEmpty(RingbufferHandle ringbufferHandle);
bool ringbuffer_isFull(RingbufferHandle ringbufferHandle);
void ringbuffer_add(RingbufferHandle ringbufferHandle, const void* value);
bool ringbuffer_get(RingbufferHandle ringbufferHandle, void* pValue, int32_t index);

#endif /* RINGBUFFER_H */