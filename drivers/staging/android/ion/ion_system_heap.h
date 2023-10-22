/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */
#include <soc/qcom/secure_buffer.h>
#include "ion.h"

#ifndef _ION_SYSTEM_HEAP_H
#define _ION_SYSTEM_HEAP_H

#ifndef CONFIG_ALLOC_BUFFERS_IN_4K_CHUNKS
#if defined(CONFIG_IOMMU_IO_PGTABLE_ARMV7S)
static const unsigned int orders[] = {8, 4, 0};
#else
static const unsigned int orders[] = {9, 4, 0};
#endif
#else
static const unsigned int orders[] = {0};
#endif

#define NUM_ORDERS ARRAY_SIZE(orders)

#define ION_KTHREAD_NICE_VAL 10

enum ion_kthread_type {
	ION_KTHREAD_UNCACHED,
	ION_KTHREAD_CACHED,
	ION_MAX_NUM_KTHREADS
};

#ifdef CONFIG_MIGRATE_HIGHORDER
#if defined(CONFIG_IOMMU_IO_PGTABLE_ARMV7S)
static const unsigned int highorders[] = {8, 4};
#else
static const unsigned int highorders[] = {9, 4};
#endif

#define NUM_HIGHORDERS ARRAY_SIZE(highorders)
#define MIN_HIGHORDER_SZ 65536
#endif

struct ion_system_heap {
	struct ion_heap heap;
	struct ion_page_pool *uncached_pools[NUM_ORDERS];
	struct ion_page_pool *cached_pools[NUM_ORDERS];
	/* worker threads to refill the pool */
	struct task_struct *kworker[ION_MAX_NUM_KTHREADS];
	struct ion_page_pool *secure_pools[VMID_LAST][NUM_ORDERS];
#ifdef CONFIG_MIGRATE_HIGHORDER
	// must check this order size of pool
	struct ion_page_pool *highorder_uncached_pools[NUM_HIGHORDERS];
	struct ion_page_pool *highorder_cached_pools[NUM_HIGHORDERS];
#endif
	/* Prevents unnecessary page splitting */
	struct mutex split_page_mutex;
};

struct page_info {
	struct page *page;
	bool from_pool;
	unsigned int order;
	struct list_head list;
};

int order_to_index(unsigned int order);

void free_buffer_page(struct ion_system_heap *heap,
		      struct ion_buffer *buffer, struct page *page,
		      unsigned int order);

#endif /* _ION_SYSTEM_HEAP_H */
