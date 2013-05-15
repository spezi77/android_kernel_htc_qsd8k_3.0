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

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/export.h>
#include <linux/dma-fence.h>

static DEFINE_SPINLOCK(fence_lock);

/**
 * dma_buf_attach_fence - Attach a fence to a dma-buf.
 *
 * @buf: the dma-buf to attach to
 * @fence: the fence to attach
 *
 * A fence can only be attached to a single dma-buf.  The dma-buf takes
 * ownership of the fence, which is unref'd when the fence is signaled.
 * The fence takes a reference to the dma-buf so the buffer will not be
 * freed while there is a pending fence.
 */
int dma_buf_attach_fence(struct dma_buf *buf, struct dma_fence *fence)
{
	if (WARN_ON(!buf || !fence || fence->buf))
		return -EINVAL;

	spin_lock(&fence_lock);
	get_dma_buf(buf);
	fence->buf = buf;
	list_add(&fence->list_node, &buf->fence_list);
	spin_unlock(&fence_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(dma_buf_attach_fence);

/**
 * dma_fence_signal - Signal a fence.
 *
 * @fence:  The fence to signal
 *
 * All registered callbacks will be called directly (synchronously) and
 * all blocked waters will be awoken.
 */
int dma_fence_signal(struct dma_fence *fence)
{
	unsigned long flags;
	int ret;

	if (WARN_ON(!fence || !fence->buf))
		return -EINVAL;

	spin_lock_irqsave(&fence_lock, flags);
	list_del(&fence->list_node);
	spin_unlock_irqrestore(&fence_lock, flags);

	dma_buf_put(fence->buf);
	fence->buf = NULL;

	ret = fence->ops->signal(fence);

	dma_fence_put(fence);

	return ret;
}
EXPORT_SYMBOL_GPL(dma_fence_signal);

/**
 * dma_buf_get_fence - Get the most recent pending fence attached to the
 * dma-buf.
 *
 * @buf: the dma-buf whose fence to get
 *
 * If this returns NULL, there are no pending fences.  Otherwise this
 * takes a reference to the returned fence, so the caller must later
 * call dma_fence_put() to release the reference.
 */
struct dma_fence *dma_buf_get_fence(struct dma_buf *buf)
{
	struct dma_fence *fence = NULL;

	if (WARN_ON(!buf))
		return ERR_PTR(-EINVAL);

	spin_lock(&fence_lock);
	if (!list_empty(&buf->fence_list)) {
		fence = list_first_entry(&buf->fence_list,
				struct dma_fence, list_node);
		dma_fence_get(fence);
	}
	spin_unlock(&fence_lock);

	return fence;
}
EXPORT_SYMBOL_GPL(dma_buf_get_fence);

static void release_fence(struct kref *kref)
{
	struct dma_fence *fence =
			container_of(kref, struct dma_fence, refcount);

	WARN_ON(waitqueue_active(&fence->event_queue) || fence->buf);

	kfree(fence);
}

/**
 * dma_fence_put - Release a reference to the fence.
 */
void dma_fence_put(struct dma_fence *fence)
{
	WARN_ON(!fence);
	kref_put(&fence->refcount, release_fence);
}
EXPORT_SYMBOL_GPL(dma_fence_put);

/**
 * dma_fence_get - Take a reference to the fence.
 *
 * In most cases this is used only internally by dma-fence.
 */
void dma_fence_get(struct dma_fence *fence)
{
	WARN_ON(!fence);
	kref_get(&fence->refcount);
}
EXPORT_SYMBOL_GPL(dma_fence_get);

/**
 * dma_fence_add_callback - Add a callback to be called when the fence
 * is signaled.
 *
 * @fence: The fence to wait on
 * @cb: The callback to register
 *
 * Any number of callbacks can be registered to a fence, but a callback
 * can only be registered to once fence at a time.
 *
 * Note that the callback can be called from an atomic context.  If
 * fence is already signaled, this function will return -EINVAL (and
 * *not* call the callback)
 */
int dma_fence_add_callback(struct dma_fence *fence,
		struct dma_fence_cb *cb)
{
	int ret = -EINVAL;

	if (WARN_ON(!fence || !cb))
		return -EINVAL;

	spin_lock(&fence_lock);
	if (fence->buf) {
		dma_fence_get(fence);
		cb->fence = fence;
		ret = fence->ops->add_callback(fence, cb);
	}
	spin_unlock(&fence_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(dma_fence_add_callback);

/**
 * dma_fence_cancel_callback - Remove a previously registered callback.
 *
 * @cb: The callback to unregister
 *
 * The callback will not be called after this function returns, but could
 * be called before this function returns.
 */
int dma_fence_cancel_callback(struct dma_fence_cb *cb)
{
	struct dma_fence *fence;
	int ret = -EINVAL;

	if (WARN_ON(!cb))
		return -EINVAL;

	fence = cb->fence;

	spin_lock(&fence_lock);
	if (fence) {
		ret = fence->ops->cancel_callback(cb);
		cb->fence = NULL;
		dma_fence_put(fence);
	}
	spin_unlock(&fence_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(dma_fence_cancel_callback);

/**
 * dma_fence_wait - Wait for a fence to be signaled.
 *
 * @fence: The fence to wait on
 * @interruptible: if true, do an interruptible wait
 * @timeout: timeout, in jiffies
 */
int dma_fence_wait(struct dma_fence *fence, bool interruptible, long timeout)
{
	if (WARN_ON(!fence))
		return -EINVAL;

	return fence->ops->wait(fence, interruptible, timeout);
}
EXPORT_SYMBOL_GPL(dma_fence_wait);

int __dma_fence_wake_func(wait_queue_t *wait, unsigned mode,
		int flags, void *key)
{
	struct dma_fence_cb *cb =
			container_of(wait, struct dma_fence_cb, base);
	int ret;

	ret = cb->func(cb, cb->fence);
	dma_fence_put(cb->fence);
	cb->fence = NULL;

	return ret;
}
EXPORT_SYMBOL_GPL(__dma_fence_wake_func);

/*
 * Helpers intended to be used by the ops of the dma_fence implementation:
 *
 * NOTE: helpers and fxns intended to be used by other dma-fence
 * implementations are not exported..  I'm not really sure if it makes
 * sense to have a dma-fence implementation that is itself a module.
 */

void __dma_fence_init(struct dma_fence *fence, struct dma_fence_ops *ops)
{
	WARN_ON(!ops || !ops->signal || !ops->add_callback ||
			!ops->cancel_callback || !ops->wait);

	kref_init(&fence->refcount);
	fence->ops = ops;
	init_waitqueue_head(&fence->event_queue);
}

int dma_fence_helper_signal(struct dma_fence *fence)
{
	wake_up_all(&fence->event_queue);
	return 0;
}

int dma_fence_helper_add_callback(struct dma_fence *fence,
		struct dma_fence_cb *cb)
{
	add_wait_queue(&fence->event_queue, &cb->base);
	return 0;
}

int dma_fence_helper_cancel_callback(struct dma_fence_cb *cb)
{
	struct dma_fence *fence = cb->fence;
	remove_wait_queue(&fence->event_queue, &cb->base);
	return 0;
}

int dma_fence_helper_wait(struct dma_fence *fence, bool interruptible,
		long timeout)
{
	int ret;

	if (interruptible) {
		ret = wait_event_interruptible_timeout(fence->event_queue,
				!fence->buf, timeout);
	} else {
		ret = wait_event_timeout(fence->event_queue,
				!fence->buf, timeout);
	}

	return ret;
}

/*
 * Pure sw implementation for dma-fence.  The CPU always gets involved.
 */

static struct dma_fence_ops sw_fence_ops = {
		.signal          = dma_fence_helper_signal,
		.add_callback    = dma_fence_helper_add_callback,
		.cancel_callback = dma_fence_helper_cancel_callback,
		.wait            = dma_fence_helper_wait,
};

/**
 * dma_fence_create - Create a simple sw-only fence.
 *
 * This fence only supports signaling from/to CPU.  Other implementations
 * of dma-fence can be used to support hardware to hardware signaling, if
 * supported by the hardware, and use the dma_fence_helper_* functions for
 * compatibility with other devices that only support sw signaling.
 */
struct dma_fence *dma_fence_create(void)
{
	struct dma_fence *fence;

	fence = kzalloc(sizeof(struct dma_fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	__dma_fence_init(fence, &sw_fence_ops);

	return fence;
}
EXPORT_SYMBOL_GPL(dma_fence_create);
