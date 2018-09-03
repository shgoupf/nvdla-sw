/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation; or, when distributed
 * separately from the Linux kernel or incorporated into other
 * software packages, subject to the following license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright 2016, 2018 International Business Machines
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdarg.h>

#include <libsnap.h>
#include <snap_tools.h>
#include <snap_s_regs.h>
#include <snap_nvdla_constants.h>

#include <nvdla_interface.h>
#include <nvdla_capi.h>
#include <nvdla_ioctl.h>

static uint32_t CURRENT_ADDR_BAR       = 0x00000000;
static const uint32_t ADDR_BAR_MASK    = 0x0002FC00;
static const uint32_t ADDR_OFFSET_MASK = 0x000003FC;
static const uint32_t ADDR_OFFSET_NUM_BIT = 10;

struct nvdla_device* global_nvdla_dev = NULL;

int verbose_flag = 1;
int _dbg_flag = 1;

#ifdef DLA_LARGE_CONFIG
static struct nvdla_config nvdla_config_os_initial = {
    .atom_size = 32,
    .bdma_enable = true,
    .rubik_enable = true,
    .weight_compress_support = true,
};
#endif

#ifdef DLA_SMALL_CONFIG
static struct nvdla_config nvdla_config_small = {
    .atom_size = 8,
    .bdma_enable = false,
    .rubik_enable = false,
    .weight_compress_support = false,
};
#endif

void dla_debug (const char* str, ...)
{
    va_list args;
    va_start (args, str);
    //vprintf (pr_fmt (str), args);
    vprintf (str, args);
    va_end (args);
}

void dla_info (const char* str, ...)
{
    va_list args;
    va_start (args, str);
    vprintf (str, args);
    va_end (args);
}

void dla_warn (const char* str, ...)
{
    va_list args;
    va_start (args, str);
    vprintf (str, args);
    va_end (args);
}

void dla_error (const char* str, ...)
{
    va_list args;
    va_start (args, str);
    vprintf (str, args);
    va_end (args);
}

void* dla_memset (void* src, int ch, uint64_t len)
{
    return memset (src, ch, len);
}

void* dla_memcpy (void* dest, const void* src, uint64_t len)
{
    return memcpy (dest, src, len);
}

int64_t dla_get_time_us (void)
{
    //return ktime_get_ns() / NSEC_PER_USEC;
    //return ktime_get_ns();
    return 0;
}

uint32_t dla_snap_bar_handle (struct snap_card* h, uint32_t addr)
{
    uint32_t bar = addr & ADDR_BAR_MASK;
    uint32_t offset = addr & ADDR_OFFSET_MASK;
    // Keep DLA selection bit (bit 8) high
    uint32_t config_data = (0x100 | (bar>>ADDR_OFFSET_NUM_BIT));

    dla_debug ("BAR handle: Addr = 'h%04x, BAR = 'h%x, OFFSET= 'h%x\n",
            addr, bar, offset);

    if (bar != CURRENT_ADDR_BAR) {
        CURRENT_ADDR_BAR = bar;
        snap_mmio_write32(h, ACTION_CONFIG, config_data);
        dla_debug ("Reg Write BAR: Addr = 'h%04x, Data = 'h%x\n",
                ACTION_CONFIG, config_data);
    }

    return offset;
}

void dla_reg_write (void* driver_context, uint32_t addr, uint32_t reg)
{
    struct nvdla_device* nvdla_dev =
        (struct nvdla_device*)driver_context;
    uint32_t register_offset = 0;

    if (!nvdla_dev) {
        return;
    }

    register_offset = 
        dla_snap_bar_handle(nvdla_dev->snap_card_handle, addr);

    snap_mmio_write32 (nvdla_dev->snap_card_handle,
                       register_offset,
                       reg);

    dla_debug ("Reg Write: Addr = 'h%04x, Data = 'h%x\n", addr, reg);
}

uint32_t dla_reg_read (void* driver_context, uint32_t addr)
{
    struct nvdla_device* nvdla_dev =
        (struct nvdla_device*)driver_context;
    uint32_t register_offset = 0;
    uint32_t data = 0;

    if (!nvdla_dev) {
        return 0;
    }

    register_offset = 
        dla_snap_bar_handle(nvdla_dev->snap_card_handle, addr);

    snap_mmio_read32 (nvdla_dev->snap_card_handle,
                      register_offset,
                      &data);

    dla_debug ("Reg Read: Addr = 'h%04x, Data = 'h%x\n", addr, data);
    return data;
}

static uint32_t nvdla_engine_isr (int32_t irq, void* data)
{
    //unsigned long flags;
    struct nvdla_device* nvdla_dev = (struct nvdla_device*)data;

    if (!nvdla_dev) {
        //return IRQ_NONE;
        return 1;
    }

    //spin_lock_irqsave (&nvdla_dev->nvdla_lock, flags);
    dla_isr_handler (nvdla_dev->engine_context);
    //complete (&nvdla_dev->event_notifier);
    //spin_unlock_irqrestore (&nvdla_dev->nvdla_lock, flags);

    //return IRQ_HANDLED;
    return 0;
}

static int32_t dla_read_dma_address (void* driver_context, void* task_data,
                                     int16_t index, void* dst)
{
    int32_t ret = 0;
    struct nvdla_mem_handle* handles;
    //dma_addr_t* phys_addr = (dma_addr_t*) (dst);
    uint64_t* addr = (uint64_t*) (dst);
    //struct nvdla_device* nvdla_dev =
    //    (struct nvdla_device*)driver_context;
    struct nvdla_task* task = (struct nvdla_task*)task_data;

    if (index == -1 || index > (int32_t)task->num_addresses) {
        return -EINVAL;
    }

    handles = (struct nvdla_mem_handle*)task->address_list;
    //ret = nvdla_gem_dma_addr (nvdla_dev->drm, task->file,
    //                          handles[index].handle,
    //                          phys_addr);

    /* Add offset to IOVA address */
    //*phys_addr = *phys_addr + handles[index].offset;
    //*addr = handles[index].offset;
    *addr = (__u64)(handles[index].handle) + handles[index].offset;

    return ret;
}

static int32_t dla_read_cpu_address (void* driver_context, void* task_data,
                                     int16_t index, void* dst)
{
    uint64_t* temp = (uint64_t*)dst;
    struct nvdla_task* task = (struct nvdla_task*)task_data;

    if (index == -1 || index > (int32_t)task->num_addresses) {
        return -EINVAL;
    }

    *temp = (uint64_t)index;
    return 0;
}

int32_t dla_get_dma_address (void* driver_context, void* task_data,
                             int16_t index, void* dst_ptr,
                             uint32_t destination)
{
    int32_t ret = 0;

    if (destination == DESTINATION_PROCESSOR) {
        ret = dla_read_cpu_address (driver_context, task_data,
                                    index, dst_ptr);
    } else if (destination == DESTINATION_DMA) {
        ret = dla_read_dma_address (driver_context, task_data,
                                    index, dst_ptr);
    } else {
        ret = -EINVAL;
    }

    return ret;
}

int32_t dla_data_write (void* driver_context, void* task_data,
                        void* src, uint64_t dst,
                        uint32_t size, uint64_t offset)
{
    int32_t ret;
    uint32_t i;
    void* ptr = NULL;
    //struct dma_buf* buf;
    struct nvdla_mem_handle* handles;
    struct nvdla_task* task = (struct nvdla_task*)task_data;

    handles = task->address_list;
    //ptr = handles[0].offset;
    ptr = handles[dst].handle;

    if (!ptr) {
        pr_err ("%s: invalid memory handles %#llx\n", __func__,
                handles[0].offset);
        ret = -ENOMEM;

        return ret;
    }

    //buf = dma_buf_get (handles[dst].handle);

    //if (IS_ERR (buf)) {
    //    pr_err ("%s: Failed get dma_buf for handle=%d\n", __func__,
    //            handles[dst].handle);
    //    return -EFAULT;
    //}

    //ret = dma_buf_begin_cpu_access (buf, DMA_BIDIRECTIONAL);

    //if (ret) {
    //    goto put_dma_buf;
    //}

    //ptr = dma_buf_vmap (buf);

    //if (!ptr) {
    //    pr_err ("%s: Failed to vmap dma_buf for handle=%d\n", __func__,
    //            handles[dst].handle);
    //    ret = -ENOMEM;
    //    goto end_cpu_access;
    //}

    memcpy ((void*) ((uint8_t*)ptr + offset), src, size);
    dla_debug("%s: write data from %#llx to %#llx(%#llx) size %d\n", __PRETTY_FUNCTION__, 
            src, ptr, offset, size);

    i = 0;
    while (i < size) {
        dla_debug("%s: writing data from %#llx to %#llx(%#llx) with %#x\n", __PRETTY_FUNCTION__,
                src, ptr, offset, *((uint8_t*)(ptr+offset)));
        i++;
        ptr++;
        src++;
    }

//    dma_buf_vunmap (buf, ptr);
//
//end_cpu_access:
//    dma_buf_end_cpu_access (buf, DMA_BIDIRECTIONAL);
//
//put_dma_buf:
//    dma_buf_put (buf);
//
    return ret;
}

int32_t dla_data_read (void* driver_context, void* task_data,
                       uint64_t src, void* dst,
                       uint32_t size, uint64_t offset)
{
    int32_t ret = 0;
    void* ptr = NULL;
    //struct dma_buf* buf;
    struct nvdla_mem_handle* handles;
    struct nvdla_task* task = (struct nvdla_task*)task_data;

    handles = task->address_list;

    dla_debug ("%s: handle %#llx, offset %#llx, num of addresses %d.\n",
            __PRETTY_FUNCTION__,
            handles[src].handle, handles[src].offset, task->num_addresses);

    //ptr = handles[src].offset;
    ptr = handles[src].handle;

    if (!ptr) {
        pr_err ("%s: invalid memory handles %#llx\n", __func__,
                handles[0].offset);
        ret = -ENOMEM;

        return ret;
    }

    //buf = dma_buf_get (handles[src].handle);

    //if (IS_ERR (buf)) {
    //    pr_err ("%s: Failed get dma_buf for handle=%d\n", __func__,
    //            handles[src].handle);
    //    return -EFAULT;
    //}

    //ret = dma_buf_begin_cpu_access (buf, DMA_BIDIRECTIONAL);

    //if (ret) {
    //    goto put_dma_buf;
    //}

    //ptr = dma_buf_vmap (buf);

    //if (!ptr) {
    //    pr_err ("%s: Failed to vmap dma_buf for handle=%d\n", __func__,
    //            handles[src].handle);
    //    ret = -ENOMEM;
    //    goto end_cpu_access;
    //}

    memcpy (dst, (void*) (((uint8_t*)ptr) + offset), size);

//    dma_buf_vunmap (buf, ptr);
//
//end_cpu_access:
//    dma_buf_end_cpu_access (buf, DMA_BIDIRECTIONAL);
//
//put_dma_buf:
//    dma_buf_put (buf);

    return ret;
}

int32_t nvdla_task_submit (struct nvdla_device* nvdla_dev, struct nvdla_task* task)
{
    int32_t err = 0;
    uint32_t task_complete = 0;
    uint32_t i = 0;

    snap_action_start((void*)nvdla_dev->snap_card_handle);

    nvdla_dev->task = task;

    //// debug the address content
    //for (i = 0; i < task->num_addresses; i++) {
    //    if (i == 5) {
    //        dla_debug ("%s: address %#llx, \n", __PRETTY_FUNCTION__,
    //                task->address_list[i].handle);
    //        __hexdump(stdout, task->address_list[i].handle, 2048);
    //    }
    //}

    err = dla_execute_task (nvdla_dev->engine_context, (void*)task, nvdla_dev->config_data);

    if (err) {
        pr_err ("Task execution failed\n");
        return err;
    }

    //// debug the address content
    //for (i = 0; i < task->num_addresses; i++) {
    //    if (i == 5) {
    //        dla_debug ("%s: address %#llx, \n", __PRETTY_FUNCTION__,
    //                task->address_list[i].handle);
    //        __hexdump(stdout, task->address_list[i].handle, 2048);
    //    }
    //}

    dla_debug ("%s: Wait for task complete\n", __PRETTY_FUNCTION__);

    while (1) {
        //unsigned long flags;

        //wait_for_completion (&nvdla_dev->event_notifier);

        //spin_lock_irqsave (&nvdla_dev->nvdla_lock, flags);

        dla_debug ("%s: Waiting interrupt\n", __PRETTY_FUNCTION__);
        snap_action_wait_interrupt((void*)nvdla_dev->snap_card_handle, NULL, 10000);
        dla_debug ("%s: Done Waiting interrupt\n", __PRETTY_FUNCTION__);

        //// debug the address content
        //for (i = 0; i < task->num_addresses; i++) {
        //    if (i == 5) {
        //        dla_debug ("%s: address %#llx, \n", __PRETTY_FUNCTION__,
        //                task->address_list[i].handle);
        //        __hexdump(stdout, task->address_list[i].handle, 2048);
        //    }
        //}

        dla_isr_handler (nvdla_dev->engine_context);

        err = dla_process_events (nvdla_dev->engine_context, &task_complete);

        //spin_unlock_irqrestore (&nvdla_dev->nvdla_lock, flags);

        if (err || task_complete) {
            break;
        }
    }

    dla_debug ("%s: Task complete\n", __PRETTY_FUNCTION__);
    dla_clear_task (nvdla_dev->engine_context);

    return err;
}

///* driver probe and init */
//static const struct of_device_id nvdla_of_match[] = {
//    {
//        .compatible = "nvidia,nvdla_os_initial",
//        .data = &nvdla_config_os_initial,
//    },
//    {
//        .compatible = "nvidia,nvdla_2",
//        .data = &nvdla_config_small,
//    },
//    { },
//};

int32_t nvdla_probe (struct snap_card* snap)
{
    //char device[64];
    int32_t err = 0;
    //struct resource* res;
    //struct device* dev = &snap->dev;
    //const struct of_device_id* match;

    dla_debug ("%s: Creating NVDLA device on snap card.\n", __PRETTY_FUNCTION__);
    //if (!snap->dev.of_node) {
    //    return -EINVAL;
    //}

    //match = of_match_device (nvdla_of_match, &snap->dev);

    //if (!match) {
    //    pr_err ("Missing DT entry!\n");
    //    return -EINVAL;
    //}

    //nvdla_dev = devm_kzalloc (dev, sizeof (*nvdla_dev), GFP_KERNEL);
    //global_nvdla_dev = malloc (sizeof (*global_nvdla_dev));
    global_nvdla_dev = alloc_mem (64, sizeof (*global_nvdla_dev));
    dla_debug ("%s: global_nvdla_dev allocated @ %#llx.\n", global_nvdla_dev, __PRETTY_FUNCTION__);

    if (!global_nvdla_dev) {
        return -ENOMEM;
    }

    //platform_set_drvdata (snap, nvdla_dev);
    global_nvdla_dev->snap_card_handle = snap;
    //nvdla_dev->config_data = (struct nvdla_config*)match->data;
    // TODO: support nvdla small config for now, need to make it
    //       configurable via command line options.
#ifdef DLA_SMALL_CONFIG
    global_nvdla_dev->config_data = &nvdla_config_small;
#elif DLA_LARGE_CONFIG
    global_nvdla_dev->config_data = &nvdla_config_os_initial;
#else
    dla_debug ("%s: invalid configuration, please choose DLA_SMALL_CONFIG or DLA_LARGE_CONFIG.\n", __PRETTY_FUNCTION__);
    return -EINVAL;
#endif

    //init_completion (&nvdla_dev->event_notifier);

    //res = platform_get_resource (snap, IORESOURCE_MEM, 0);
    //nvdla_dev->base = devm_ioremap_resource (&snap->dev, res);
    // TODO: use snap as the base struct.
    global_nvdla_dev->base = snap;

    //if (IS_ERR (nvdla_dev->base)) {
    //    return PTR_ERR (nvdla_dev->base);
    //}

    //res = platform_get_resource (snap, IORESOURCE_IRQ, 0);

    //if (!res) {
    //    dev_err (&snap->dev, "no irq resource\n");
    //    return -EINVAL;
    //}

    //nvdla_dev->irq = res->start;

    //err = devm_request_irq (&snap->dev, nvdla_dev->irq,
    //                        nvdla_engine_isr, 0,
    //                        dev_name (&snap->dev), nvdla_dev);

    //if (err) {
    //    return err;
    //}

    dla_register_driver (&global_nvdla_dev->engine_context, (void*)global_nvdla_dev);
    dla_clear_task (global_nvdla_dev->engine_context);

    dla_debug ("%s: Done creating NVDLA device on snap card.\n", __PRETTY_FUNCTION__);
    //err = nvdla_drm_probe (nvdla_dev);

    //if (err) {
    //    dev_err (&snap->dev, "failed to register drm device\n");
    //}

    return err;
}

//static int32_t __exit nvdla_remove (struct snap_card* snap)
static int32_t nvdla_remove (struct snap_card* snap)
{
    // struct nvdla_device* nvdla_dev = dev_get_drvdata (&snap->dev);

    snap_card_free(snap);

    return 0;
}

