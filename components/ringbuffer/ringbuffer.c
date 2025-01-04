// https://github.com/klushund/embedded-systems-mit-risc-v/tree/main/components/components/ringbuffer
// Thx for the code! Adjusted it to take any type of data, not just floats.

#include "ringbuffer.h"

typedef struct _Ringbuffer_ {
	uint32_t writeOffset;
	uint32_t count;
	size_t size;
	size_t element_size;
	void* values;
} Ringbuffer;

RingbufferHandle ringbuffer_create(uint32_t size, size_t element_size) {
	Ringbuffer* pRingbuffer = malloc(sizeof(Ringbuffer));
	if (pRingbuffer == NULL) {
		return RINGBUFFER_ERROR_OUTOFMEMORY;
	}
	pRingbuffer->values = malloc(size * element_size);
	if (pRingbuffer->values == NULL) {
		free(pRingbuffer);
		return RINGBUFFER_ERROR_OUTOFMEMORY;
	}
	pRingbuffer->size = size;
	pRingbuffer->element_size = element_size;
	pRingbuffer->writeOffset = 0;
	pRingbuffer->count = 0;
	ESP_LOGI(RINGBUFFER_TAG, "Ringbuffer of size %lu created", size);
	return (RingbufferHandle) pRingbuffer;
}

void ringbuffer_destroy(RingbufferHandle* pRingbufferHandle) {
	Ringbuffer* pRingbuffer = (Ringbuffer*) *pRingbufferHandle;
	free(pRingbuffer->values);
	free(*((Ringbuffer**)pRingbufferHandle));
	*pRingbufferHandle = 0;
}

void ringbuffer_clear(RingbufferHandle ringbufferHandle) {
	Ringbuffer* pRingbuffer = (Ringbuffer*) ringbufferHandle;
	pRingbuffer->writeOffset = 0;
	pRingbuffer->count = 0;
}

bool ringbuffer_isEmpty(RingbufferHandle ringbufferHandle) {
	Ringbuffer* pRingbuffer = (Ringbuffer*) ringbufferHandle;
	return (pRingbuffer->count == 0);
}

bool ringbuffer_isFull(RingbufferHandle ringbufferHandle) {
	Ringbuffer* pRingbuffer = (Ringbuffer*) ringbufferHandle;
	return (pRingbuffer->count >= pRingbuffer->size);
}

void ringbuffer_add(RingbufferHandle ringbufferHandle, const void* value) {
	Ringbuffer* pRingbuffer = (Ringbuffer*) ringbufferHandle;
	void* dest = (char*)pRingbuffer->values + (pRingbuffer->writeOffset * pRingbuffer->element_size);
	memcpy(dest, value, pRingbuffer->element_size);
	pRingbuffer->writeOffset = (pRingbuffer->writeOffset + 1) % pRingbuffer->size;
	if (pRingbuffer->count < pRingbuffer->size) {
		pRingbuffer->count += 1;
	}
	ESP_LOGV(RINGBUFFER_TAG, "Added value at position %lu", pRingbuffer->writeOffset);
}

bool ringbuffer_get(RingbufferHandle ringbufferHandle, void* pValue, int32_t index) {
	Ringbuffer* pRingbuffer = (Ringbuffer*) ringbufferHandle;
	if (index >= pRingbuffer->count) {
		return false;
	}
	if (index < 0) {
		index = (pRingbuffer->writeOffset + index + pRingbuffer->size) % pRingbuffer->size;
	}

	int32_t offs = pRingbuffer->writeOffset - pRingbuffer->count + index;
	if (offs < 0) {
		offs += pRingbuffer->size;
	} else {
		offs %= pRingbuffer->size;
	}
	void* src = (void*)pRingbuffer->values + (offs * pRingbuffer->element_size);
	memcpy(pValue, src, pRingbuffer->element_size);
	return true;
}