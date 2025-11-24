#include "xy6020l.h"


bool xy6020l::read_hold_register_data(uint16_t holding_reg_start_addr, uint16_t no_of_register_to_read){
  if (!serialHandle) return false;
  if (no_of_register_to_read == 0 || no_of_register_to_read > 30) return false;
  
  serialHandle->setTimeout(10);

  //setup a temporary tx buffer
  uint8_t txDataBuf[8];
  txDataBuf[0] = DEFAULT_SLAVE_ADDRESS;
  txDataBuf[1] = FUNC_CODE_READ_HOLD_REG;
  txDataBuf[2] = holding_reg_start_addr >> 8;
  txDataBuf[3] = holding_reg_start_addr & 0xFF;
  txDataBuf[4] = no_of_register_to_read >> 8;
  txDataBuf[5] = no_of_register_to_read & 0xFF;

  uint16_t crc16 = crc16_calculator(txDataBuf, 6);

  txDataBuf[6] = crc16 & 0xFF;
  txDataBuf[7] = crc16 >> 8;

  size_t txDataSent = serialHandle->write(txDataBuf, sizeof(txDataBuf));
  if(txDataSent != sizeof(txDataBuf)) return false;

  uint8_t expectedRxBytes = (no_of_register_to_read * 2) + 5;
  memset(response_temp_buf, 0, expectedRxBytes);

  vTaskDelay(pdMS_TO_TICKS(5));
  
  size_t bytesRead = serialHandle->readBytes(response_temp_buf, expectedRxBytes);
  if (bytesRead != expectedRxBytes) return false;
  if (response_temp_buf[1] & 0x80) return false;

  uint16_t rxCRC16 = (response_temp_buf[expectedRxBytes - 1] << 8) | (response_temp_buf[expectedRxBytes - 2]);
  if (crc16_calculator(response_temp_buf, expectedRxBytes - 2) != rxCRC16) return false;

  return true;
}

bool xy6020l::write_a_single_register(uint16_t reg_address, uint16_t value){
  if (!serialHandle) return false;
  
  serialHandle->setTimeout(10);

  uint8_t txDataBuf[8];
  txDataBuf[0] = _slave_address;
  txDataBuf[1] = FUNC_CODE_WRITE_SINGLE_HOLD_REG;
  txDataBuf[2] = reg_address >> 8;
  txDataBuf[3] = reg_address & 0xFF;
  txDataBuf[4] = value >> 8;
  txDataBuf[5] = value & 0xFF;

  uint16_t crc16 = crc16_calculator(txDataBuf, 6);

  txDataBuf[6] = crc16 & 0xFF;
  txDataBuf[7] = crc16 >> 8;

  size_t txDataSent = serialHandle->write(txDataBuf, sizeof(txDataBuf));
  if(txDataSent != sizeof(txDataBuf)) return false;

  uint8_t expectedRxBytes = 8;

  memset(response_temp_buf, 0, expectedRxBytes);

  vTaskDelay(pdMS_TO_TICKS(5));
  
  size_t bytesRead = serialHandle->readBytes(response_temp_buf, expectedRxBytes);
  if (bytesRead != expectedRxBytes) return false;
  if (response_temp_buf[1] & 0x80) return false;

  uint16_t rxCRC16 = (response_temp_buf[expectedRxBytes - 1] << 8) | (response_temp_buf[expectedRxBytes - 2]);

  if(crc16_calculator(response_temp_buf, expectedRxBytes - 2) != rxCRC16) return false;

  return true;
}

bool xy6020l::write_multiple_registers(uint16_t holding_reg_start_addr, uint16_t no_of_register_to_write, uint8_t data_byte_count, uint8_t *value_buf){
  if (!serialHandle) return false;

  if (no_of_register_to_write == 0 || no_of_register_to_write > 14) return false;
  if (data_byte_count != no_of_register_to_write * 2) return false;
  if (!value_buf) return false;

  
  serialHandle->setTimeout(10);

  uint8_t txDataBuf[9 + data_byte_count];
  txDataBuf[0] = _slave_address;
  txDataBuf[1] = FUNC_CODE_WRITE_MULTIPLE_HOLD_REG;
  txDataBuf[2] = holding_reg_start_addr >> 8;
  txDataBuf[3] = holding_reg_start_addr & 0xFF;
  txDataBuf[4] = no_of_register_to_write >> 8;
  txDataBuf[5] = no_of_register_to_write & 0xFF;
  txDataBuf[6] = data_byte_count;

  for(int data = 0; data < data_byte_count; data++){
    txDataBuf[7 + data] = value_buf[data];
  }

  uint16_t crc16 = crc16_calculator(txDataBuf, 7 + data_byte_count);
  txDataBuf[7 + data_byte_count] = crc16 & 0xFF;
  txDataBuf[8 + data_byte_count] = crc16 >> 8;

  size_t txDataSent = serialHandle->write(txDataBuf, sizeof(txDataBuf));
  if(txDataSent != sizeof(txDataBuf)) return false;

  uint8_t expectedRxBytes = 8;
  memset(response_temp_buf, 0, expectedRxBytes);

  vTaskDelay(pdMS_TO_TICKS(5));
  
  size_t bytesRead = serialHandle->readBytes(response_temp_buf, expectedRxBytes);
  if (bytesRead != expectedRxBytes) return false;
  if (response_temp_buf[1] & 0x80) return false;

  uint16_t rxCRC16 = (response_temp_buf[expectedRxBytes - 1] << 8) | (response_temp_buf[expectedRxBytes - 2]);

  if(crc16_calculator(response_temp_buf, expectedRxBytes - 2) != rxCRC16) return false;

  return true;
}

bool xy6020l::get_all_hold_regs(){
  bool response = read_hold_register_data(HREG_IDX_CV, 30);

  if (!response) return false;

  for (int data = 0; data < 60; data++){
    all_hold_reg_data[data] = response_temp_buf[data + 3];
  }

  return true;
}

uint16_t xy6020l::crc16_calculator(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}