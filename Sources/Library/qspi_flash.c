#include <stm32h7xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_qspi.h>
#include "qspi_flash.h"

#include <string.h>
#include <stdbool.h>

static QSPI_HandleTypeDef hqspi;
// static MDMA_HandleTypeDef hmdma;

static volatile uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch;

static spi_flash_info sf_info;
static bool memory_mapped = false;

int qspi_flash_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_QSPI_CLK_ENABLE();
  __HAL_RCC_MDMA_CLK_ENABLE();

  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**QUADSPI GPIO Configuration
  PG6     ------> QUADSPI_BK1_NCS
  PF6    ------> QUADSPI_BK1_IO3
  PF7     ------> QUADSPI_BK1_IO2
  PD11     ------> QUADSPI_BK1_IO0
  PB2     ------> QUADSPI_CLK
  PF9     ------> QUADSPI_BK1_IO1
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = QSPI_FLASH_SIZE;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;

  HAL_QSPI_DeInit(&hqspi);
  if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(QUADSPI_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(QUADSPI_IRQn);

  return HAL_OK;
}

int qspi_flash_hp_modoe(void) {
  QSPI_CommandTypeDef command = {0};

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  command.Instruction = 0xA3;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_NONE;
  command.DummyCycles = 24;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static int qspi_flash_write_enable(void) {
  QSPI_CommandTypeDef command = {0};
  QSPI_AutoPollingTypeDef config;

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  command.Instruction = WRITE_ENABLE_CMD;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_NONE;
  command.DummyCycles = 0;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  config.Match = 0x02;
  config.Mask = 0x02;
  config.MatchMode = QSPI_MATCH_MODE_AND;
  config.StatusBytesSize = 1;
  config.Interval = 0x10;
  config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  command.Instruction = READ_STATUS_REG_CMD;
  command.DataMode = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&hqspi, &command, &config,
                           HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static int qspi_flash_atuo_polling_mem_ready(void) {
  QSPI_CommandTypeDef command = {0};
  QSPI_AutoPollingTypeDef config = {0};

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = READ_STATUS_REG_CMD;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_1_LINE;
  command.DummyCycles = 0;

  config.Mask = 0x01;
  config.Match = 0x00;
  config.MatchMode = QSPI_MATCH_MODE_AND;
  config.StatusBytesSize = 1;
  config.Interval = 0x10;
  config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi, &command, &config,
                           HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

int qspi_flash_read_status1(uint8_t *status) {
  QSPI_CommandTypeDef command;

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = READ_STATUS_REG_CMD;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_1_LINE;
  command.DummyCycles = 0;
  command.NbData = 1;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, status, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

int qspi_flash_read_status2(uint8_t *status) {
  QSPI_CommandTypeDef command;

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = READ_STATUS2_REG_CMD;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_1_LINE;
  command.DummyCycles = 0;
  command.NbData = 1;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, status, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

int qspi_flash_write_status(uint8_t status1_val, uint8_t status2_val) {
  QSPI_CommandTypeDef command = {0};
  uint8_t buf[2];

  TxCplt = 0;

  qspi_flash_write_enable();

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_ONLY_FIRST_CMD;

  command.Instruction = WRITE_STATUS_REG_CMD;
  command.DummyCycles = 0;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 2;
  buf[0] = status1_val;
  buf[1] = status2_val;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Transmit(&hqspi, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (qspi_flash_atuo_polling_mem_ready() != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

int qspi_flash_write_status2(uint8_t status2_val) {
  QSPI_CommandTypeDef command = {0};
  uint8_t buf[2];

  TxCplt = 0;

  qspi_flash_write_enable();

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_ONLY_FIRST_CMD;

  command.Instruction = WRITE_STATUS2_REG_CMD;
  command.DummyCycles = 0;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 1;
  buf[0] = status2_val;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Transmit(&hqspi, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (qspi_flash_atuo_polling_mem_ready() != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

uint32_t qspi_flash_read_id(void) {
  uint32_t id = 0;
  QSPI_CommandTypeDef command;
  uint8_t buf[3];

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = READ_ID_CMD2;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.DataMode = QSPI_DATA_1_LINE;
  command.DummyCycles = 0;
  command.NbData = 3;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return 0;
  }

  if (HAL_QSPI_Receive(&hqspi, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return 0;
  }

  id = (buf[0] << 16) | (buf[1] << 8) | buf[2];

  return id;
}

int qspi_flash_config(void) {
  uint32_t id;
  uint8_t status1, status2;
  id = qspi_flash_read_id();
  if (id == 0) {
    return HAL_ERROR;
  }

  // WINBOND GIGADEVICE
  if (((id >> 16 & 0xFF) == 0xEF) || ((id >> 16 & 0xFF) == 0xC8)) {
    qspi_flash_read_status2(&status2);
    // quad enable bit
    if ((status2 & 0x02) == 0) {
      qspi_flash_read_status1(&status1);
      // qspi_flash_write_status(status1, status2 | 0x02);
      qspi_flash_write_status2(status2 | 0x02);
    }
  }
  if ((id >> 16 & 0xFF) == 0xC8) {
    qspi_flash_hp_modoe();
  }

  switch (id & 0xFF) {
    case 0x20:
      sf_info.block_count = 1024;
      sf_info.desc = "64M bytes";
      break;
    case 0x19:
      sf_info.block_count = 512;
      sf_info.desc = "32M bytes";
      break;
    case 0x18:
      sf_info.block_count = 256;
      sf_info.desc = "16M bytes";
      break;
    case 0x17:
      sf_info.block_count = 128;
      sf_info.desc = "8M bytes";
      break;
    case 0x16:
      sf_info.block_count = 64;
      sf_info.desc = "4M bytes";
      break;
    case 0x15:
      sf_info.block_count = 32;
      sf_info.desc = "2M bytes";
      break;
    case 0x14:
      sf_info.block_count = 16;
      sf_info.desc = "1M bytes";
      break;
    case 0x13:
      sf_info.block_count = 8;
      sf_info.desc = "512K bytes";
      break;
    case 0x12:
      sf_info.block_count = 4;
      sf_info.desc = "256K bytes";
      break;
    case 0x11:
      sf_info.block_count = 2;
      sf_info.desc = "128K bytes";
      break;
    default:
      sf_info.lock = 0;
      sf_info.desc = "unknown flash";
      return HAL_ERROR;
  }
  sf_info.page_size = 256;
  sf_info.sector_size = 0x1000;
  sf_info.sector_count = sf_info.block_count * 16;
  sf_info.page_count =
      (sf_info.sector_count * sf_info.sector_size) / sf_info.page_size;
  sf_info.block_size = sf_info.sector_size * 16;
  sf_info.capacity_in_kilobyte =
      (sf_info.sector_count * sf_info.sector_size) / 1024;
  return HAL_OK;
}

int qspi_flash_erase_sector(uint32_t address) {
  QSPI_CommandTypeDef command = {0};

  CmdCplt = 0;

  qspi_flash_write_enable();

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = SUBSECTOR_ERASE_4_BYTE_ADDR_CMD;
  command.AddressMode = QSPI_ADDRESS_1_LINE;
  command.Address = address;
  command.DataMode = QSPI_DATA_NONE;
  command.DummyCycles = 0;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (qspi_flash_atuo_polling_mem_ready() != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

int qspi_flash_erase_block_64k(uint32_t address) {
  QSPI_CommandTypeDef command = {0};

  CmdCplt = 0;

  if (memory_mapped) {
    qspi_flash_quit_memory_mapped();
  }

  qspi_flash_write_enable();

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = SECTOR_64K_ERASE_CMD;
  command.AddressMode = QSPI_ADDRESS_1_LINE;
  command.Address = address;
  command.DataMode = QSPI_DATA_NONE;
  command.DummyCycles = 0;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (qspi_flash_atuo_polling_mem_ready() != HAL_OK) {
    return HAL_ERROR;
  }

  if (memory_mapped) {
    qspi_flash_memory_mapped();
  }
  return HAL_OK;
}

int qspi_flash_erase_chip(void) {
  QSPI_CommandTypeDef command = {0};

  CmdCplt = 0;

  qspi_flash_write_enable();

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = BULK_ERASE_CMD;
  command.AddressMode = QSPI_DATA_NONE;
  command.Address = 0;
  command.DataMode = QSPI_DATA_NONE;
  command.DummyCycles = 0;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (qspi_flash_atuo_polling_mem_ready() != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

int qspi_flash_write_page(uint8_t *data, uint32_t address, uint16_t len) {
  QSPI_CommandTypeDef command = {0};

  TxCplt = 0;

  if (memory_mapped) {
    qspi_flash_quit_memory_mapped();
  }

  qspi_flash_write_enable();

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_ONLY_FIRST_CMD;

  command.Instruction = QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD;
  command.DummyCycles = 0;
  command.AddressMode = QSPI_ADDRESS_1_LINE;
  command.DataMode = QSPI_DATA_4_LINES;
  command.NbData = len;
  command.Address = address;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Transmit(&hqspi, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }
  // while (TxCplt == 0)
  //   ;
  // TxCplt = 0;

  if (qspi_flash_atuo_polling_mem_ready() != HAL_OK) {
    return HAL_ERROR;
  }

  if (memory_mapped) {
    qspi_flash_memory_mapped();
  }

  return HAL_OK;
}

int qspi_flash_write_buffer_unsafe(uint8_t *data, uint32_t address,
                                   uint32_t len) {
  uint32_t page_remain = 0;

  page_remain = sf_info.page_size - address % sf_info.page_size;

  while (len) {
    page_remain = len > page_remain ? page_remain : len;
    qspi_flash_write_page(data, address, page_remain);
    len -= page_remain;
    data += page_remain;
    address += page_remain;
    page_remain = sf_info.page_size;
  }

  return HAL_OK;
}

int qspi_flash_read_buffer(uint8_t *data, uint32_t address, uint32_t len) {
  QSPI_CommandTypeDef command = {0};

  RxCplt = 0;

  if (memory_mapped) {
    qspi_flash_quit_memory_mapped();
  }

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD;
  command.DummyCycles = 6;
  command.AddressMode = QSPI_ADDRESS_4_LINES;
  command.DataMode = QSPI_DATA_4_LINES;
  command.NbData = len;
  command.Address = address;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return HAL_ERROR;
  }

  // while (RxCplt == 0)
  //   ;
  // RxCplt = 0;

  if (memory_mapped) {
    qspi_flash_memory_mapped();
  }

  return HAL_OK;
}

int qspi_flash_memory_mapped(void) {
  QSPI_CommandTypeDef command = {0};
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD;
  command.AddressMode = QSPI_ADDRESS_4_LINES;
  command.DataMode = QSPI_DATA_4_LINES;
  command.DummyCycles = 6;

  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod = 0;

  if (HAL_QSPI_MemoryMapped(&hqspi, &command, &s_mem_mapped_cfg) != HAL_OK) {
    return HAL_ERROR;
  }
  memory_mapped = true;
  return HAL_OK;
}

int qspi_flash_quit_memory_mapped(void) {
  HAL_QSPI_DeInit(&hqspi);
  hqspi.Instance->CCR = 0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
 * @brief  Command completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi) { CmdCplt++; }

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi) { RxCplt++; }

/**
 * @brief  Tx Transfer completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) { TxCplt++; }

/**
 * @brief  Status Match callbacks
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi) { StatusMatch++; }

/**
 * @brief  This function handles QUADSPI interrupt request.
 * @param  None
 * @retval None
 */
void QUADSPI_IRQHandler(void) { HAL_QSPI_IRQHandler(&hqspi); }

/**
 * @brief  This function handles QUADSPI DMA interrupt request.
 * @param  None
 * @retval None
 */

