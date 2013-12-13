/*
 * RapidIO buffer management support
 *
 * Copyright 2013 Prodrive B.V.
 * Jerry Jacobs <jerry.jacobs@prodrive.nl>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>

#include <linux/rio.h>
#include <linux/rio_buff.h>

#include <linux/mm.h>
#include <linux/slab.h>

/**
 * __rio_buff_offset() - Get offset of buf from base
 * @base:	Buffer base address
 * @buf:	Buffer slot (RIO_MAX_MSG_SIZE aligned)
 */
int __rio_buff_offset(void *base, void *buf)
{
	uint32_t *base_addr = (uint32_t *)cpu_to_be32(base);
	uint32_t *buf_addr  = (uint32_t *)cpu_to_be32(buf);
	int offset = 0;

	/* TODO this is not very beautifull,
		should get rid of sizeof(void *).
		Hence this only works on 32bit machines */
	offset = ((buf_addr - base_addr)
		* sizeof(void *)) / RIO_MAX_MSG_SIZE;

	return offset;
}

/**
 * __rio_buff_state() - Get state of buffer slot
 * @buffer:	RapidIO messages buffer
 * @slot:	Slot on buffer
 */
int __rio_buff_state(struct rio_buffer *buffer, void *slot)
{
	uint32_t offset = 0;

	offset = __rio_buff_offset(buffer->buffer, slot);
	if (offset >= buffer->size)
		return -ENOMEM;

	return buffer->state[offset];
}

/**
 * __rio_buff_set_state() - Set state of buffer slot
 * @buffer:	RapidIO messages buffer
 * @slot:	Slot in buffer
 * @state:	New state of slot
 */
int __rio_buff_set_state(struct rio_buffer *buffer,
	void *slot, enum rio_buff_state state)
{
	int offset = 0;

	if (!buffer->buffer)
		return -EINVAL;

	offset = __rio_buff_offset(buffer->buffer, slot);
	if (offset < 0 || offset >= buffer->size)
		return -ENOMEM;

	buffer->state[offset] = state;

	return 0;
}

/**
 * rio_buff_malloc() - Allocate a RapidIO buffer that hold size messages
 * @buffer:	RapidIO messages buffer
 * @size:	Number of messages to allocate in the buffer
 */
int rio_buff_malloc(struct rio_buffer *buffer, size_t size)
{
	unsigned int page;

	/* Allocate ringbuffer slots */
	buffer->buffer = kzalloc((size + 1) * PAGE_SIZE, GFP_KERNEL);
	if (!buffer->buffer)
		return -ENOMEM;

	/* Allocate slot states */
	buffer->state = kzalloc(size * sizeof(*buffer->state), GFP_KERNEL);
	if (!buffer->state)
		goto err;

	/* Round it up to the page boundary */
	buffer->buffer_area = (int *)((((unsigned long)buffer->buffer) + PAGE_SIZE - 1) & PAGE_MASK);

	/* Set pages reserved */
	for (page = 0; page < size; page++)
		SetPageReserved(virt_to_page((void *)(((unsigned long)buffer->buffer_area) + page)));

	buffer->size = size;
	buffer->free = size;

	spin_lock_init(&buffer->lock);

	pr_debug("rio-buff: Created buffer at 0x%p\n", buffer->buffer);

	return 0;

err:
	kfree(buffer->buffer);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(rio_buff_malloc);

/**
 * rio_buff_free() - Free RapidIO messages buffer
 * @buffer:	RapidIO messages buffer
 */
int rio_buff_free(struct rio_buffer *buffer)
{
	unsigned int page;

	/* Set pages unreserved */
	for (page = 0; page < buffer->size; page++)
		ClearPageReserved(virt_to_page((void *)(((unsigned long)buffer->buffer_area) + page)));

	kfree(buffer->buffer_area);
	kfree(buffer->state);

	return 0;
}
EXPORT_SYMBOL_GPL(rio_buff_free);

/**
 * rio_buff_request_slot() - Request message slot in RapidIO messages buffer
 * @buffer:	RapidIO messages buffer
 * @slot:	Requested buffer slot
 * @state:	State of requested buffer slot
 *
 * When a buffer is requested the state is changed to RIO_BUFF_INUSE.
 *
 * Return: -EINVAL,  invalid rio_buffer
 *         -EXFULL,  no slot found at state RIO_BUFF_FREE
 *         -ENODATA, no slot found at state RIO_BUFF_FILLED
 */
int rio_buff_request_slot(struct rio_buffer *buffer,
	void **slot,
	enum rio_buff_state state)
{
	int ret = 0;
	int n;
	int cstate;
	unsigned long flags;
	void *buf;

	if (!buffer || !slot)
		return -EINVAL;

	local_irq_save(flags);
	if (!spin_trylock(&buffer->lock)) {
		local_irq_restore(flags);
		return -EBUSY;
	}

	if (buffer->free == 0) {
		spin_unlock_irqrestore(&buffer->lock, flags);
		return -ENOMEM;
	}

	/* Search for free or filled slot */
	for (n = 0; n < buffer->size; n++) {

		buf = buffer->buffer + (n * RIO_MAX_MSG_SIZE);
		cstate  = __rio_buff_state(buffer, buf);

		if (state == cstate) {
			*slot = buf;
			pr_debug("rio-buff: Found requested slot %p, with state %d\n",
				buf, cstate);
			__rio_buff_set_state(buffer, buf, RIO_BUFF_INUSE);
			buffer->free--;
			ret = 0;
			goto out;
		}
	}

	*slot = NULL;

	/* No requested slot found */
	if (state == RIO_BUFF_FREE)
		ret = -EXFULL;
	else
		ret = -ENODATA;

out:
	spin_unlock_irqrestore(&buffer->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(rio_buff_request_slot);

/**
 * rio_buff_release_slot() - Release message slot and set at state
 * @buffer:	RapidIO messages buffer
 * @slot:	Buffer slot to release
 * @state:	New buffer slot state
 */
int rio_buff_release_slot(struct rio_buffer *buffer,
	void *slot,
	enum rio_buff_state state)
{
	int ret = 0;
	unsigned long flags;

	if (buffer == NULL || slot == NULL)
		return -EINVAL;

	local_irq_save(flags);
	if (!spin_trylock(&buffer->lock)) {
		local_irq_restore(flags);
		return -EBUSY;
	}

	ret = __rio_buff_set_state(buffer, slot, state);
	if (ret < 0) {
		ret = -EFAULT;
		goto out;
	}

	pr_debug("rio-buff: Release %p with state %d\n", slot, state);

	buffer->free++;

out:
	spin_unlock_irqrestore(&buffer->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(rio_buff_release_slot);

/**
 * rio_buff_release_all() - Releases all messages slot to FREE
 * @buffer:	RapidIO messages buffer
 */
int rio_buff_release_all(struct rio_buffer *buffer)
{
	int n;

	if (buffer == NULL)
		return -EINVAL;

	buffer->free = buffer->size;
	for (n = 0; n < buffer->size; n++)
		buffer->state[n] = RIO_BUFF_FREE;

	return 0;
}
EXPORT_SYMBOL_GPL(rio_buff_release_all);

/**
 * rio_buff_vma_info() - VMA information for buffer slot
 * @buffer:	RapidIO messages buffer
 * @slot:	Buffer slot
 * @info:	Buffer vma private information
 */
int rio_buff_vma_info(struct rio_buffer *buffer,
	void *slot,
	struct rio_buff_vma_info **info)
{
	int state = 0;

	if (!*info)
		return -EINVAL;

	state = __rio_buff_state(buffer, slot);
	if (state == RIO_BUFF_INVALID)
		return -ENOMEM;

	(*info)->buffer = buffer;
	(*info)->slot   = slot;
	(*info)->state  = state;

	return 0;
}
EXPORT_SYMBOL_GPL(rio_buff_vma_info);

