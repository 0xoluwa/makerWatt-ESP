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

  txDataBuf[0] = DEFAULT_SLAVE_ADDRESS;
  txDataBuf[1] = 0x3; //fucntion code to read register
  txDataBuf[2] = register_start_add >> 8;
  txDataBuf[3] = register_start_add & 0xFF;
  txDataBuf[4] = no_of_register >> 8;
  txDataBuf[5] = no_of_register & 0xFF;

  uint16_t crc16 = calculateCRC(txDataBuf, sizeof(txDataBuf));

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
  uint32_t previousTime = millis();
  uint8_t dataBuf[65];
  serialHandle->readBytes(dataBuf, 65);
  Serial.printf("It took %d ms", (millis() - previousTime));
  int count = 0;
  while(count < sizeof(dataBuf)){
      Serial.printf("[0x"); Serial.print(dataBuf[count], HEX); Serial.printf("] ");
      count ++;
  }
}

void setup(){
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(1000);
  data_to_modbus_framing_03(&Serial2, HREG_IDX_CV, 30);
}

void loop(){

}