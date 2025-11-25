#include <Arduino.h>
#include "components/xy6020l.h"


uint16_t calculateCRC(uint8_t *data, uint8_t length) {
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


void data_to_modbus_framing_03(Stream *serialHandle, uint16_t register_start_add, uint16_t no_of_register){
  uint8_t txDataBuf[6];
  serialHandle->setTimeout(100);

  txDataBuf[0] = DEFAULT_SLAVE_ADDRESS;
  txDataBuf[1] = 0x3; //fucntion code to read register
  txDataBuf[2] = register_start_add >> 8;
  txDataBuf[3] = register_start_add & 0xFF;
  txDataBuf[4] = no_of_register >> 8;
  txDataBuf[5] = no_of_register & 0xFF;

  uint16_t crc16 = calculateCRC(txDataBuf, sizeof(txDataBuf));
  uint8_t expectedRxBytes = (no_of_register * 2) + 5;

  Serial.print("DATA TO BE SENT IS: ");
  for(int i = 0; i < 6; i++){
    Serial.printf("[0x%d] ", txDataBuf[i]);
  }
  Serial.printf("[0x%d]", crc16);
  //Serial.printf("[0x%d] \n", crc16 & 0xFF);

  size_t txBufSent = serialHandle->write(txDataBuf, sizeof(txDataBuf));
  size_t crcSent = serialHandle->write(crc16 & 0xFF);
  size_t crcSent1 = serialHandle->write(crc16 >> 8);

  Serial.printf("Sent txBuf: %d          Sent crc %d and %d\n", txBufSent, crcSent, crcSent1);
  
  Serial.println("DATA SENT");
  uint8_t dataBuf[65];
  uint32_t previousTime = millis();
  serialHandle->readBytes(dataBuf, expectedRxBytes);
  Serial.printf("It took %d ms", (millis() - previousTime));
  int count = 0;
  while(count < expectedRxBytes){
      Serial.printf("[0x"); Serial.print(dataBuf[count], HEX); Serial.printf("] ");
      count ++;
  }
}

void write_a_single_register(Stream *serialHandle, uint16_t reg_address, uint16_t value){
  serialHandle->setTimeout(100);

  uint8_t expectedRxBytes = 8;
  uint8_t response_temp_buf[70];

  uint8_t txDataBuf[8];
  txDataBuf[0] = DEFAULT_SLAVE_ADDRESS;
  txDataBuf[1] = FUNC_CODE_WRITE_SINGLE_HOLD_REG;
  txDataBuf[2] = reg_address >> 8;
  txDataBuf[3] = reg_address & 0xFF;
  txDataBuf[4] = value >> 8;
  txDataBuf[5] = value & 0xFF;

  uint16_t crc16 = calculateCRC(txDataBuf, 6);

  txDataBuf[6] = crc16 & 0xFF;
  txDataBuf[7] = crc16 >> 8;

  uint32_t previousTime = millis();
  size_t txDataSent = serialHandle->write(txDataBuf, sizeof(txDataBuf));
  if(txDataSent != sizeof(txDataBuf)) {
    Serial.println("The amount of data succesfully sent is not equal to data fed to serial");
    return;
  }
  
  size_t bytesRead = serialHandle->readBytes(response_temp_buf, expectedRxBytes);

  Serial.printf("It took exactly %dms for the operation \n", millis() - previousTime);

  for(int i = 0; i < expectedRxBytes; i++){
    Serial.printf("[0x%d] ", response_temp_buf[i]);
  }

  if (response_temp_buf[1] & 0x80){
    Serial.println("EXCEPTION OCCURED!!!!");
    return;
  }

  if (bytesRead != expectedRxBytes){
    Serial.println("Bytes received from serial is not equal to expected bytes ");
    return;
  }
  uint16_t rxCRC16 = (response_temp_buf[expectedRxBytes - 1] << 8) | (response_temp_buf[expectedRxBytes - 2]);

  if(calculateCRC(response_temp_buf, expectedRxBytes - 2) != rxCRC16){
    Serial.println("Received checksum is not equalt to tx checksum");
    return;
  }

  Serial.println("Tx succesfully completed");

}

void setup(){
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(1000);
  write_a_single_register(&Serial2, HREG_IDX_CV, 20);
}

void loop(){
  if (Serial.available()){
    uint16_t voltage = Serial.parseInt();
    Serial.printf("start register set to %d volt \n", voltage);
    data_to_modbus_framing_03(&Serial2, voltage, 30 - voltage);
  }
  delay(50);
}