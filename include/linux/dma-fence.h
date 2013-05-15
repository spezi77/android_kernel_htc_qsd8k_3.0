/*
 * Fence mechanism for dma-buf to allow for asynchronous dma access
 *
 * Copyright (C) 2012 Texas Instruments
 * Author: Rob Clark <rob.clark@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DMA_FENCE_H__
#define __DMA_FENCE_H__

#include <linux/err.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-buf.h>

struct dma_fence;
struct dma_fence_ops;
struct dma_fence_cb;

struct dma_fence {
	struct kref refcount;
	struct dma_fence_ops *ops;
	wait_queue_head_t event_queue;
	struct list_head list_node;   /* for dmabuf's fence_list */

	/*
	 * This is initialized when the fence is attached to a dma-buf,
	 * and NULL'd before it is signaled.  A fence can be attached to
	 * at most one dma-buf at a time.
	 */
	struct dma_buf *buf;
};

typedef int (*dma_fence_func_t)(struct dma_fence_cb *cb,
		struct dma_fence *fence);

struct dma_fence_ops {
	int (*signal)(struct dma_fence *fence);
	int (*add_callback)(struct dma_fence *fence, struct dma_fence_cb *cb);
	int (*cancel_callback)(struct dma_fence_cb *cb);
	int (*wait)(struct dma_fence *fence, bool interruptible, long timeout);
};

struct dma_fence_cb {
	wait_queue_t base;
	dma_fence_func_t func;

	/*
	 * This is initialized when the cb is added, and NULL'd when it
	 * is canceled or expired, so can be used to for error checking
	 * if the cb is already pending.  A dma_fence_cb can be pending
	 * on at most one fence at a time.
	 */
	struct dma_fence *fence;
};

int __dma_fence_wake_func(wait_queue_t *wait, unsigned mode,
		int flags, void *key);

#define DMA_FENCE_CB_INITIALIZER(cb_func) {                \
		.base = { .func = __dma_fence_wake_func },  \
		.func = (cb_func),                          \
	}

#define DECLARE_DMA_FENCE(name, cb_func)                   \
		struct dma_fence_cb name = DMA_FENCE_CB_INITIALIZER(cb_func)


/*
 * TODO does it make sense to be able to enable dma-fence without dma-buf,
 * or visa versa?
 */
#ifdef CONFIG_DMA_SHARED_BUFFER

int dma_buf_attach_fence(struct dma_buf *buf, struct dma_fence *fence);
struct dma_fence *dma_buf_get_fence(struct dma_buf *buf);

/* create a basic (pure sw) fence: */
struct dma_fence *dma_fence_create(void);

/* intended to be used by other dma_fence implementations: */
void __dma_fence_init(struct dma_fence *fence, struct dma_fence_ops *ops);

void dma_fence_get(struct dma_fence *fence);
void dma_fence_put(struct dma_fence *fence);
int dma_fence_signal(struct dma_fence *fence);

int dma_fence_add_callback(struct dma_fence *fence,
		struct dma_fence_cb *cb);
int dma_fence_cancel_callback(struct dma_fence_cb *cb);
int dma_fence_wait(struct dma_fence *fence, bool interruptible, long timeout);

/* helpers intended to be used by the ops of the dma_fence implementation: */
int dma_fence_helper_signal(struct dma_fence *fence);
int dma_fence_helper_add_callback(struct dma_fence *fence,
		struct dma_fence_cb *cb);
int dma_fence_helper_cancel_callback(struct dma_fence_cb *cb);
int dma_fence_helper_wait(struct dma_fence *fence, bool interruptible,
		long timeout);

#else
// TODO
#endif /* CONFIG_DMA_SHARED_BUFFER */

#endif /* __DMA_FENCE_H__ */
