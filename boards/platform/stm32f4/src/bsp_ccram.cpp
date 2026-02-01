/*###########################################################
# Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "bsp_ccram.h"
#include <string.h>

#if MATH_USE_CCRAM

/* ==================== CCRAM内存管理器 ==================== */

// 内部结构体定义（只在.cpp中）
typedef struct {
    uint8_t* base_addr;
    size_t total_size;
    size_t used_size;
    size_t block_size;
    uint32_t block_count;
    uint32_t free_blocks;
    uint32_t* block_map;
    uint32_t allocations;
} CCRAM_Manager_t;

static CCRAM_Manager_t ccram_manager = {};
static MathPerformanceMode current_perf_mode = MATH_PERF_AUTO;

// 简单的内存分配器
static struct {
    uint8_t* next;
    size_t remaining;
} ccram_simple_allocator = {};

/* ==================== CCRAM API实现 ==================== */

bool CCRAM_Init(void) {
    if (ccram_manager.base_addr != NULL) {
        return true;
    }

    ccram_manager.base_addr = (uint8_t*)CCRAM_BASE_ADDR;
    ccram_manager.total_size = CCRAM_SIZE;
    ccram_manager.used_size = 0;
    ccram_manager.allocations = 0;

    // 使用简单的顺序分配器
    ccram_simple_allocator.next = ccram_manager.base_addr;
    ccram_simple_allocator.remaining = CCRAM_SIZE;

    // 清零CCRAM区域的前1KB
    memset(ccram_manager.base_addr, 0, 1024);

    return true;
}

void* CCRAM_Malloc(size_t size) {
    if (size == 0 || !ccram_simple_allocator.next) {
        return NULL;
    }

    // 对齐到8字节
    size = (size + 7) & ~7;

    if (ccram_simple_allocator.remaining < size) {
        return NULL;
    }

    void* ptr = ccram_simple_allocator.next;
    ccram_simple_allocator.next += size;
    ccram_simple_allocator.remaining -= size;

    ccram_manager.used_size += size;
    ccram_manager.allocations++;

    return ptr;
}

void* CCRAM_Calloc(size_t num, size_t size) {
    size_t total_size = num * size;
    void* ptr = CCRAM_Malloc(total_size);

    if (ptr) {
        CCRAM_Memset(ptr, 0, total_size);
    }

    return ptr;
}

// 新增对齐分配函数
void* CCRAM_Malloc_Aligned(size_t size, size_t alignment) {
    if (size == 0 || !ccram_simple_allocator.next) {
        return NULL;
    }

    // 确保对齐值是2的幂
    if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
        return NULL;
    }

    // 计算实际需要的空间（原始大小 + 对齐填充 + 存储原始地址的空间）
    size_t needed_size = size + alignment - 1 + sizeof(void*);
    if (ccram_simple_allocator.remaining < needed_size) {
        return NULL;
    }

    // 找到对齐后的地址
    uint8_t* raw_ptr = ccram_simple_allocator.next;
    uint8_t* aligned_ptr = (uint8_t*)(((uintptr_t)(raw_ptr + sizeof(void*)) + alignment - 1) & ~(alignment - 1));

    // 存储原始指针位置
    ((void**)aligned_ptr)[-1] = raw_ptr;

    // 更新分配器状态
    size_t used_size = aligned_ptr - raw_ptr + size;
    ccram_simple_allocator.next += used_size;
    ccram_simple_allocator.remaining -= used_size;

    ccram_manager.used_size += used_size;
    ccram_manager.allocations++;

    return aligned_ptr;
}

void* CCRAM_Calloc_Aligned(size_t num, size_t size, size_t alignment) {
    size_t total_size = num * size;
    void* ptr = CCRAM_Malloc_Aligned(total_size, alignment);

    if (ptr) {
        CCRAM_Memset(ptr, 0, total_size);
    }

    return ptr;
}

void CCRAM_Free(void* ptr) {
    // 简单分配器不支持释放单个块
    (void)ptr;
}

void CCRAM_GetStats(size_t* used, size_t* free, uint32_t* fragments) {
    if (used) *used = ccram_manager.used_size;
    if (free) *free = ccram_manager.total_size - ccram_manager.used_size;
    if (fragments) *fragments = 0;
}

bool CCRAM_IsInCCRAM(const void* ptr) {
    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t base = (uintptr_t)ccram_manager.base_addr;
    return (addr >= base && addr < base + ccram_manager.total_size);
}

void MathSetPerformanceMode(MathPerformanceMode mode) {
    current_perf_mode = mode;
}

MathPerformanceMode MathGetPerformanceMode(void) {
    return current_perf_mode;
}

void CCRAM_Memset(void* ptr, int value, size_t num) {
    if (ptr && num > 0) {
        // 直接在CCRAM中执行内存设置
        memset(ptr, value, num);
    }
}

#endif // MATH_USE_CCRAM
