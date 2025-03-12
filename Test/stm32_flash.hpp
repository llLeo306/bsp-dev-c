#pragma once

#include "flash.hpp"
#include "main.h"

/* base address of the flash sectors */
#define ADDR_FLASH_SECTOR_0 \
  ((uint32_t)0x08000000) /* Base address of Sector 0, 16 K bytes   */
#define ADDR_FLASH_SECTOR_1 \
  ((uint32_t)0x08004000) /* Base address of Sector 1, 16 K bytes   */
#define ADDR_FLASH_SECTOR_2 \
  ((uint32_t)0x08008000) /* Base address of Sector 2, 16 K bytes   */
#define ADDR_FLASH_SECTOR_3 \
  ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 K bytes   */
#define ADDR_FLASH_SECTOR_4 \
  ((uint32_t)0x08010000) /* Base address of Sector 4, 64 K bytes   */
#define ADDR_FLASH_SECTOR_5 \
  ((uint32_t)0x08020000) /* Base address of Sector 5, 128 K bytes  */
#define ADDR_FLASH_SECTOR_6 \
  ((uint32_t)0x08040000) /* Base address of Sector 6, 128 K bytes  */
#define ADDR_FLASH_SECTOR_7 \
  ((uint32_t)0x08060000) /* Base address of Sector 7, 128 K bytes  */
#define ADDR_FLASH_SECTOR_8 \
  ((uint32_t)0x08080000) /* Base address of Sector 8, 128 K bytes  */
#define ADDR_FLASH_SECTOR_9 \
  ((uint32_t)0x080A0000) /* Base address of Sector 9, 128 K bytes  */
#define ADDR_FLASH_SECTOR_10 \
  ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 K bytes */
#define ADDR_FLASH_SECTOR_11 \
  ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 K bytes */

static uint32_t stm32_get_sector(uint32_t address) {
  uint32_t sector = 0;

  if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
    sector = FLASH_SECTOR_0;
  } else if ((address < ADDR_FLASH_SECTOR_2) &&
             (address >= ADDR_FLASH_SECTOR_1)) {
    sector = FLASH_SECTOR_1;
  } else if ((address < ADDR_FLASH_SECTOR_3) &&
             (address >= ADDR_FLASH_SECTOR_2)) {
    sector = FLASH_SECTOR_2;
  } else if ((address < ADDR_FLASH_SECTOR_4) &&
             (address >= ADDR_FLASH_SECTOR_3)) {
    sector = FLASH_SECTOR_3;
  } else if ((address < ADDR_FLASH_SECTOR_5) &&
             (address >= ADDR_FLASH_SECTOR_4)) {
    sector = FLASH_SECTOR_4;
  } else if ((address < ADDR_FLASH_SECTOR_6) &&
             (address >= ADDR_FLASH_SECTOR_5)) {
    sector = FLASH_SECTOR_5;
  } else if ((address < ADDR_FLASH_SECTOR_7) &&
             (address >= ADDR_FLASH_SECTOR_6)) {
    sector = FLASH_SECTOR_6;
  } else if ((address < ADDR_FLASH_SECTOR_8) &&
             (address >= ADDR_FLASH_SECTOR_7)) {
    sector = FLASH_SECTOR_7;
  } else if ((address < ADDR_FLASH_SECTOR_9) &&
             (address >= ADDR_FLASH_SECTOR_8)) {
    sector = FLASH_SECTOR_8;
  } else if ((address < ADDR_FLASH_SECTOR_10) &&
             (address >= ADDR_FLASH_SECTOR_9)) {
    sector = FLASH_SECTOR_9;
  } else if ((address < ADDR_FLASH_SECTOR_11) &&
             (address >= ADDR_FLASH_SECTOR_10)) {
    sector = FLASH_SECTOR_10;
  } else {
    sector = FLASH_SECTOR_11;
  }
  return sector;
}

static uint32_t stm32_get_sector_size(uint32_t sector) {
  switch (sector) {
    case ADDR_FLASH_SECTOR_0:
      return 16 * 1024;
    case ADDR_FLASH_SECTOR_1:
      return 16 * 1024;
    case ADDR_FLASH_SECTOR_2:
      return 16 * 1024;
    case ADDR_FLASH_SECTOR_3:
      return 16 * 1024;
    case ADDR_FLASH_SECTOR_4:
      return 64 * 1024;
    case ADDR_FLASH_SECTOR_5:
      return 128 * 1024;
    case ADDR_FLASH_SECTOR_6:
      return 128 * 1024;
    case ADDR_FLASH_SECTOR_7:
      return 128 * 1024;
    case ADDR_FLASH_SECTOR_8:
      return 128 * 1024;
    case ADDR_FLASH_SECTOR_9:
      return 128 * 1024;
    case ADDR_FLASH_SECTOR_10:
      return 128 * 1024;
    case ADDR_FLASH_SECTOR_11:
      return 128 * 1024;
    default:
      return 128 * 1024;
  }
}

namespace LibXR {

class STM32Flash : public Flash {
 public:
  STM32Flash(size_t start_offset, size_t min_erase_size, size_t min_write_size)
      : Flash(min_erase_size, min_write_size,
              RawData(reinterpret_cast<void*>(FLASH_BASE + start_offset),
                      2 * min_erase_size)),
        start_offset_(start_offset),
        min_erase_size_(min_erase_size) {}

  ErrorCode Erase(size_t offset, size_t size) override {
    if ((offset + size) > 2 * min_erase_size_) {
      return ErrorCode::OUT_OF_RANGE;
    }

    offset += start_offset_;

    HAL_FLASH_Unlock();

    for (size_t addr = offset; addr < offset + size;) {
      FLASH_EraseInitTypeDef erase_init = {};
      uint32_t sector_error = 0;

#if defined(FLASH_TYPEERASE_PAGES)  // STM32F1 series
      erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
      erase_init.Page = addr / FLASH_PAGE_SIZE;
      erase_init.NbPages = 1;
      erase_init.Banks = FLASH_BANK_1;
#elif defined(FLASH_TYPEERASE_SECTORS)  // STM32F4/F7/H7 series
      erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
      erase_init.Sector = stm32_get_sector(addr + FLASH_BASE);
      erase_init.NbSectors = 1;
      erase_init.Banks = FLASH_BANK_1;
#else                                   // Default case for other STM32 series
      return ErrorCode::NOT_SUPPORT;
#endif

      if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return ErrorCode::FAILED;
      }

      addr += stm32_get_sector_size(erase_init.Sector);
    }

    HAL_FLASH_Lock();
    return ErrorCode::OK;
  }

  ErrorCode Write(size_t offset, ConstRawData data) override {
    if ((offset + data.size_) > 2 * min_erase_size_) {
      return ErrorCode::OUT_OF_RANGE;
    }

    offset += start_offset_;

    HAL_FLASH_Unlock();

    uint8_t* src =
        static_cast<uint8_t*>(const_cast<void*>(data.addr_));  // NOLINT
    uint32_t addr = FLASH_BASE + offset;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    while (addr < (FLASH_BASE + offset + data.size_)) {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, *src) == HAL_OK) {
        addr += 1;
        src += 1;
      }

      else {
        HAL_FLASH_Lock();
        return ErrorCode::FAILED;
      }
    }

    HAL_FLASH_Lock();
    return ErrorCode::OK;
  }

 private:
  size_t start_offset_;
  size_t min_erase_size_;
};

}  // namespace LibXR
