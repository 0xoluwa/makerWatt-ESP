#include "xy6020l.h"


bool xy6020l::read_hold_register_data(uint16_t holding_reg_start_addr, uint16_t no_of_register_to_read){
  if (!serialHandle) return false;
  if (no_of_register_to_read == 0 || no_of_register_to_read > 30) return false;
  
  serialHandle->setTimeout(10);

  //setup a temporary tx buffer
  uint8_t txDataBuf[8];
  txDataBuf[0] = _slave_address;
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
  
  serialHandle->setTimeout(40);

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

  if (!response) return response;

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

bool xy6020l::fetch_preset(tMemory &presetStruct){
  if (presetStruct.num >= 10) return false;

  uint16_t start_address = HREG_IDX_M0 + (presetStruct.num * HREG_IDX_M_OFFSET);

  bool response = read_hold_register_data(start_address, MEM_REGS);

  if(!response) return response;

  presetStruct.VSet = (response_temp_buf[3] << 8) | response_temp_buf[4];
  presetStruct.ISet = (response_temp_buf[5] << 8) | response_temp_buf[6];
  presetStruct.sLVP = (response_temp_buf[7] << 8) | response_temp_buf[8];
  presetStruct.sOVP = (response_temp_buf[9] << 8) | response_temp_buf[10];
  presetStruct.sOCP = (response_temp_buf[11] << 8) | response_temp_buf[12];
  presetStruct.sOPP = (response_temp_buf[13] << 8) | response_temp_buf[14];
  presetStruct.sOHPh = (response_temp_buf[15] << 8) | response_temp_buf[16];
  presetStruct.sOHPm = (response_temp_buf[17] << 8) | response_temp_buf[18];

  uint16_t sOAH_low = (response_temp_buf[19] << 8) | response_temp_buf[20];
  uint16_t sOAH_high = (response_temp_buf[21] << 8) | response_temp_buf[22];
  presetStruct.sOAH = ((uint32_t)sOAH_high << 16) | sOAH_low;

  uint16_t sOWH_low = (response_temp_buf[23] << 8) | response_temp_buf[24];
  uint16_t sOWH_high = (response_temp_buf[25] << 8) | response_temp_buf[26];
  presetStruct.sOWH = ((uint32_t)sOWH_high << 16) | sOWH_low;

  presetStruct.sOTP = (response_temp_buf[27] << 8) | response_temp_buf[28];
  presetStruct.sINI = (response_temp_buf[29] << 8) | response_temp_buf[30];

  return true;
}

bool xy6020l::set_preset(tMemory &preset){
  if (preset.num >= 10) return false;
  uint16_t start_address = HREG_IDX_M0 + (preset.num * HREG_IDX_M_OFFSET);

  uint8_t temp_data_buf[MEM_REGS * 2] = {
                                      preset.VSet >> 8,
                                      preset.VSet & 0xFF,
                                      preset.ISet >> 8,
                                      preset.ISet & 0xFF,
                                      preset.sLVP >> 8,
                                      preset.sLVP & 0xFF,
                                      preset.sOVP >> 8,
                                      preset.sOVP & 0xFF,
                                      preset.sOCP >> 8,
                                      preset.sOCP & 0xFF,
                                      preset.sOPP >> 8,
                                      preset.sOPP & 0xFF,
                                      preset.sOHPh >> 8,
                                      preset.sOHPh & 0xFF,
                                      preset.sOHPm >> 8,
                                      preset.sOHPm & 0xFF,
                                      (preset.sOAH & 0xFFFF) >> 8,  //low 16 bytes
                                      (preset.sOAH & 0xFFFF) & 0xFF,
                                      (preset.sOAH >> 16) >> 8,     //upper 16 bytes
                                      (preset.sOAH >> 16) & 0xFF,
                                      (preset.sOWH & 0xFFFF) >> 8,
                                      (preset.sOWH & 0xFFFF) & 0xFF,
                                      (preset.sOWH >> 16) >> 8,
                                      (preset.sOWH >> 16) & 0xFF,
                                      preset.sOTP >> 8,
                                      preset.sOTP & 0xFF,
                                      preset.sINI >> 8,
                                      preset.sINI & 0xFF

  };

  bool response = write_multiple_registers(start_address, MEM_REGS, MEM_REGS * 2, temp_data_buf);

  if(!response) return response;

  return true;

}