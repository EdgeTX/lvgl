
/**
  ******************************************************************************
  * @file    nema_hal.c
  * @author  MCD Application Team
  * @brief   NemaGFX Interfaces and Platform Specific APIs with no OS support.
  *          This file provides NemaGFX functions to manage the following
  *          functionalities of the NemaGFX library:
  *           + Initialization and de-initialization functions
  *           + GPU2D Registers access operation functions
  *           + Buffer Creation, Destruction, MMAP and Flusing functions
  *           + Interrupt management functions
  *           + Memory management functions
  *           + Locking/Unlocking resources functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

//#include <touchgfx/hal/Config.hpp>
#include <nema_sys_defs.h>
#include <nema_core.h>

#include <assert.h>
#include <string.h>

#include <stm32h7rsxx_hal.h>

#include "tsi_malloc.h"

#define RING_SIZE                      1024 /* Ring Buffer Size in byte */
#define NEMAGFX_MEM_POOL_SIZE          16128 /* NemaGFX byte pool size in byte */
#define NEMAGFX_STENCIL_POOL_SIZE      485120 /* NemaGFX stencil buffer pool size in byte */

//LOCATION_PRAGMA_NOLOAD("Nemagfx_Memory_Pool_Buffer")
static uint8_t nemagfx_pool_mem[NEMAGFX_MEM_POOL_SIZE] __attribute__((section(".nemaPool")));// LOCATION_ATTRIBUTE_NOLOAD("Nemagfx_Memory_Pool_Buffer"); /* NemaGFX memory pool */

//LOCATION_PRAGMA_NOLOAD("Nemagfx_Stencil_Buffer")
static uint8_t nemagfx_stencil_buffer_mem[NEMAGFX_STENCIL_POOL_SIZE] __attribute__((section(".nemaBuffer")));// LOCATION_ATTRIBUTE_NOLOAD("Nemagfx_Stencil_Buffer"); /* NemaGFX stencil buffer memory */

static nema_ringbuffer_t ring_buffer_str;
volatile static int last_cl_id = -1;
GPU2D_HandleTypeDef hgpu2d;

#if (USE_HAL_GPU2D_REGISTER_CALLBACKS == 1)
static void GPU2D_CommandListCpltCallback(GPU2D_HandleTypeDef* hgpu2d, uint32_t CmdListID)
#else /* USE_HAL_GPU2D_REGISTER_CALLBACKS = 0 */
__attribute__((__used__)) void HAL_(GPU2D_HandleTypeDef* hgpu2d, uint32_t CmdListID)
#endif /* USE_HAL_GPU2D_REGISTER_CALLBACKS = 1 */
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hgpu2d);

    last_cl_id = CmdListID;
}

void HAL_GPU2D_ErrorCallback(GPU2D_HandleTypeDef *hgpu2d)
{
    uint32_t val = nema_reg_read(GPU2D_SYS_INTERRUPT); /* clear the ER interrupt */
    nema_reg_write(GPU2D_SYS_INTERRUPT, val);
    if (val & ~0xFU)
    {
        /* unrecoverable error */
//        for (;;);
    }
    /* external GPU2D cache maintenance */
    if (val & (1UL << 2))
    {
        //HAL_ICACHE_Disable();
        SCB_DisableICache();
        nema_ext_hold_deassert_imm(2);
    }
    if (val & (1UL << 3))
    {
      SCB_EnableICache();
      SCB_InvalidateICache();
//        HAL_ICACHE_Enable();
//        HAL_ICACHE_Invalidate();
        nema_ext_hold_deassert_imm(3);
    }
}

void platform_disable_cache(void)
{
  SCB_CleanInvalidateDCache();
  SCB_DisableDCache();
    nema_ext_hold_assert(2, 1);
}

void platform_invalidate_cache(void)
{
  SCB_CleanInvalidateDCache();
    nema_ext_hold_assert(3, 1);
}

int32_t nema_sys_init(void)
{
    int error_code = 0;

    __HAL_RCC_GPU2D_CLK_ENABLE();

    /* GPU2D interrupt Init */
    HAL_NVIC_SetPriority(GPU2D_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPU2D_IRQn);
    HAL_NVIC_SetPriority(GPU2D_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPU2D_ER_IRQn);
    hgpu2d.Instance = GPU2D;
    HAL_GPU2D_Init(&hgpu2d);

    /* Setup GPU2D Callback */
#if (USE_HAL_GPU2D_REGISTER_CALLBACKS == 1)||1
    /* Register Command List Comlete Callback */
    HAL_GPU2D_RegisterCommandListCpltCallback(&hgpu2d, GPU2D_CommandListCpltCallback);
#endif /* USE_HAL_GPU2D_REGISTER_CALLBACKS = 1 */

    /* Initialise Mem Space */
    error_code = tsi_malloc_init_pool_aligned(0, (void*)nemagfx_pool_mem, (uintptr_t)nemagfx_pool_mem, NEMAGFX_MEM_POOL_SIZE, 1, 8);
    assert(error_code == 0);
    error_code = tsi_malloc_init_pool_aligned(1, (void*)nemagfx_stencil_buffer_mem, (uintptr_t)nemagfx_stencil_buffer_mem, NEMAGFX_STENCIL_POOL_SIZE, 1, 8);
    assert(error_code == 0);

    /* Allocate ring_buffer memory */
    ring_buffer_str.bo = nema_buffer_create(RING_SIZE);
    assert(ring_buffer_str.bo.base_virt);

    /* Initialize Ring Buffer */
    error_code = nema_rb_init(&ring_buffer_str, 1);
    if (error_code < 0)
    {
        return error_code;
    }

    /* Reset last_cl_id counter */
    last_cl_id = 0;

    return error_code;
}

uint32_t nema_reg_read(uint32_t reg)
{
    return HAL_GPU2D_ReadRegister(&hgpu2d, reg);
}

void nema_reg_write(uint32_t reg, uint32_t value)
{
    SCB_CleanDCache();
    HAL_GPU2D_WriteRegister(&hgpu2d, reg, value);
}

int nema_wait_irq(void)
{
    return 0;
}

int nema_wait_irq_cl(int cl_id)
{
    while (last_cl_id < cl_id)
    {
        (void)nema_wait_irq();
    }

    return 0;
}

int nema_wait_irq_brk(int brk_id)
{
    while (nema_reg_read(GPU2D_BREAKPOINT) == 0U)
    {
        (void)nema_wait_irq();
    }

    return 0;
}

void nema_host_free(void* ptr)
{
    tsi_free(ptr);
}

void* nema_host_malloc(unsigned size)
{
    return tsi_malloc(size);
}

nema_buffer_t nema_buffer_create(int size)
{
    nema_buffer_t bo;
    memset(&bo, 0, sizeof(bo));
    bo.base_virt = tsi_malloc(size);
    bo.base_phys = (uint32_t)bo.base_virt;
    bo.size      = size;
    assert(bo.base_virt != 0 && "Unable to allocate memory in nema_buffer_create");

    return bo;
}

nema_buffer_t nema_buffer_create_pool(int pool, int size)
{
    nema_buffer_t bo;
    memset(&bo, 0, sizeof(bo));
    bo.base_virt = tsi_malloc_pool(pool, size);
    bo.base_phys = (uint32_t)bo.base_virt;
    bo.size      = size;
    bo.fd        = 0;
    assert(bo.base_virt != 0 && "Unable to allocate memory in nema_buffer_create_pool");

    return bo;
}

void* nema_buffer_map(nema_buffer_t* bo)
{
    return bo->base_virt;
}

void nema_buffer_unmap(nema_buffer_t* bo)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(bo);
}

void nema_buffer_destroy(nema_buffer_t* bo)
{
    if (bo->fd == -1)
    {
        return; /* Buffer weren't allocated! */
    }

    tsi_free(bo->base_virt);

    bo->base_virt = (void*)0;
    bo->base_phys = 0;
    bo->size      = 0;
    bo->fd        = -1; /* Buffer not allocated */
}

uintptr_t nema_buffer_phys(nema_buffer_t* bo)
{
    return bo->base_phys;
}

void nema_buffer_flush(nema_buffer_t* bo)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(bo);
}

int nema_mutex_lock(int mutex_id)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(mutex_id);

    return 0;
}

int nema_mutex_unlock(int mutex_id)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(mutex_id);

    return 0;
}


void GPU2D_IRQHandler(void)
{
  /* USER CODE BEGIN GPU2D_IRQn 0 */

  /* USER CODE END GPU2D_IRQn 0 */
  HAL_GPU2D_IRQHandler(&hgpu2d);
  /* USER CODE BEGIN GPU2D_IRQn 1 */

  /* USER CODE END GPU2D_IRQn 1 */
}

/**
  * @brief This function handles GPU2D Error interrupt.
  */
void GPU2D_ER_IRQHandler(void)
{
  /* USER CODE BEGIN GPU2D_ER_IRQn 0 */

  /* USER CODE END GPU2D_ER_IRQn 0 */
  HAL_GPU2D_ER_IRQHandler(&hgpu2d);
  /* USER CODE BEGIN GPU2D_ER_IRQn 1 */

  /* USER CODE END GPU2D_ER_IRQn 1 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

