/**
 * @file xy6020l.h
 * @brief UART control access to XY6020L DCDC
 *
 * This library provides an embedded and simplified ModBus implementation.
 * The data type of interfaces to physical values are in decimal, scaled almost in 0.01.
 *
 * Tested only on a Arduino Pro Micro clone from China
 *
 * Special thanks to user g-radmac for his discovery of the UART protocol!
 * References: 
 *   https://forum.allaboutcircuits.com/threads/exploring-programming-a-xy6020l-power-supply-via-modbus.197022/
 *   https://www.simplymodbus.ca/FAQ.htm
 *   
 * 
 * @author Jens Gleissberg
 * @date 2024
 * @license GNU Lesser General Public License v3.0 or later
 * 
 * @edited by: 0xoluwa
 *
 */

#ifndef xy6020l_h
#define xy6020l_h

#include "Arduino.h"

// the XY6020 provides 31 holding registers
#define HOLD_REGS 31
#define MEM_REGS 14

// Holding Register index

#define HREG_IDX_CV               0  // set voltage
#define HREG_IDX_CC               1 // set current
#define HREG_IDX_ACT_V            2  // actual voltage  0,01 V
#define HREG_IDX_ACT_C            3  // actual current   0,01 A
#define HREG_IDX_ACT_P            4  // actual output power  0,1 W
#define HREG_IDX_IN_V             5 // input voltage  0,01 V
#define HREG_IDX_OUT_CHRG         6 // output charge  0,001 Ah
#define HREG_IDX_OUT_CHRG_HIGH    7
#define HREG_IDX_OUT_ENERGY       8 // output energy  0,001 Wh
#define HREG_IDX_OUT_ENERGY_HIGH  9

#define HREG_IDX_ON_HOUR          0x0A // on time  [h]   ??
#define HREG_IDX_ON_MIN           0x0B  // on time  [min]   
#define HREG_IDX_ON_SEC           0x0C  // on time  [s] 

// temperature  0,1 °C / Fahrenheit ?
#define HREG_IDX_TEMP             0x0D
#define HREG_IDX_TEMP_EXD         0x0E

// key lock changes
#define HREG_IDX_LOCK             0x0F
#define HREG_IDX_PROTECT          0x10
#define HREG_IDX_CVCC             0x11

// output on
#define HREG_IDX_OUTPUT_ON        0x12

#define HREG_IDX_FC               0x13 //temperature symbol c or f

#define HREG_IDX_BB               0x14 //backlight control
#define HREG_IDX_SLEEP            0x15 //off screen time

//device details
#define HREG_IDX_MODEL            0x16
#define HREG_IDX_VERSION          0x17
#define HREG_IDX_SLAVE_ADD        0x18
#define HREG_IDX_BAUDRATE         0x19

//temperature offset
#define HREG_IDX_TEMP_OFS         0x1A 
#define HREG_IDX_TEMP_EXT_OFS     0x1B

#define HREG_IDX_MEMORY           0x1D //call out and set preset
// Memory register
#define HREG_IDX_M0               0x50
#define HREG_IDX_M_OFFSET         0x10
#define HREG_IDX_M_VSET           0
#define HREG_IDX_M_ISET           1
#define HREG_IDX_M_SLVP           2
#define HREG_IDX_M_SOVP           3
#define HREG_IDX_M_SOCP           4
#define HREG_IDX_M_SOPP           5
#define HREG_IDX_M_SOHPH          6
#define HREG_IDX_M_SOHPM          7
#define HREG_IDX_M_SOAHL          8
#define HREG_IDX_M_SOAHH          9
#define HREG_IDX_M_SOWHL          10
#define HREG_IDX_M_SOWHH          11
#define HREG_IDX_M_SOTP           12
#define HREG_IDX_M_SINI           13

#define DEFAULT_SLAVE_ADDRESS     0x1

#define TX_RING_BUFFER_SIZE 16

#define FUNC_CODE_READ_HOLD_REG               0x3
#define FUNC_CODE_WRITE_SINGLE_HOLD_REG       0x06
#define FUNC_CODE_WRITE_MULTIPLE_HOLD_REG     0x10

enum XY6020L_EXCEPTIONS{
    ILLEGAL_FUNCTION = 1,
    ILLEGAL_DATA_ADDRESS,
    ILLEGAL_DATA_VALUE,
    SLAVE_DEVICE_FAILURE,
    ACKNOWLEDGE,
    SLAVE_DEVICE_BUSY,
    NEGATIVE_ACKNOWLEDGE,
    MEMORY_PARITY_ERROR,
    GATEWAY_PATH_UNAVAILABLE,
    GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND
};

enum RxState { IDLE, RECEIVING, COMPLETE };

typedef struct {
  uint8_t mHregIdx;
  uint16_t mValue;
} txRingEle;

typedef struct {
    uint8_t num;
    uint16_t VSet;
    uint16_t ISet;
    uint16_t sLVP;
    uint16_t sOVP;
    uint16_t sOCP;
    uint16_t sOPP;
    uint16_t sOHPh;
    uint16_t sOHPm;
    uint32_t sOAH;
    uint32_t sOWH;
    uint16_t sOTP;
    uint16_t sINI;
} tMemory;


/**
 * @class xy6020l
 * @brief Class for controlling the XY6020L DCDC converter
 */
class xy6020l 
{
  public:
    /**
     * @brief Constructor requires an interface to serial port
     * @param serial Stream object reference (i.e., Serial1)
     * @param adr slave address of the xy device, can be change by setSlaveAdd command
     * @param txPeriod minimum period to wait for next tx message, at times < 50 ms the XY6020 does not send answers
     */
    xy6020l(Stream *serial, uint8_t addr=1) : serialHandle(serial), _slave_address(addr)
    {

    }

    void begin(){

    }
      uint16_t get_set_volt(){return (uint16_t) (all_hold_reg_data[HREG_IDX_CV*2] << 8) | (all_hold_reg_data[(HREG_IDX_CV * 2) + 1]);}
      uint16_t get_set_current(){return (uint16_t) (all_hold_reg_data[HREG_IDX_CC *2] << 8) | (all_hold_reg_data[(HREG_IDX_CC * 2) + 1]);}
      uint16_t get_actual_volt(){return (uint16_t) (all_hold_reg_data[HREG_IDX_ACT_V * 2] << 8) | (all_hold_reg_data[(HREG_IDX_ACT_V * 2) + 1]);}
      uint16_t get_actual_current(){return (uint16_t) (all_hold_reg_data[HREG_IDX_ACT_C * 2] << 8) | (all_hold_reg_data[(HREG_IDX_ACT_C * 2) + 1]);}
      uint16_t get_power(){return (uint16_t) (all_hold_reg_data[HREG_IDX_ACT_P * 2] << 8) | (all_hold_reg_data[(HREG_IDX_ACT_P * 2) + 1]);}
      uint16_t get_input_volt(){return (uint16_t) (all_hold_reg_data[HREG_IDX_IN_V * 2] << 8) | (all_hold_reg_data[(HREG_IDX_IN_V * 2) + 1]);}

      uint32_t get_amp_hour(){
        uint16_t ah_low = (uint16_t) (all_hold_reg_data[HREG_IDX_OUT_CHRG * 2] << 8) | (all_hold_reg_data[(HREG_IDX_OUT_CHRG * 2) + 1]);
        uint16_t ah_high = (uint16_t) (all_hold_reg_data[HREG_IDX_OUT_CHRG_HIGH * 2] << 8) | (all_hold_reg_data[(HREG_IDX_OUT_CHRG_HIGH * 2) + 1]);
        return (uint32_t) (ah_high << 16) | ah_low;
      }

      uint32_t get_watt_hour(){
        uint16_t ah_low = (uint16_t) (all_hold_reg_data[HREG_IDX_OUT_ENERGY * 2] << 8) | (all_hold_reg_data[(HREG_IDX_OUT_ENERGY * 2) + 1]);
        uint16_t ah_high = (uint16_t) (all_hold_reg_data[HREG_IDX_OUT_ENERGY_HIGH * 2] << 8) | (all_hold_reg_data[(HREG_IDX_OUT_ENERGY_HIGH * 2) + 1]);
        return (uint32_t) (ah_high << 16) | ah_low;
      }

      uint16_t get_output_hour(){return (uint16_t) (all_hold_reg_data[HREG_IDX_ON_HOUR * 2] << 8) | (all_hold_reg_data[(HREG_IDX_ON_HOUR * 2) + 1]);}
      uint16_t get_output_min(){return (uint16_t) (all_hold_reg_data[HREG_IDX_ON_MIN * 2] << 8) | (all_hold_reg_data[(HREG_IDX_ON_MIN * 2) + 1]);}
      uint16_t get_output_sec(){return (uint16_t) (all_hold_reg_data[HREG_IDX_ON_SEC * 2] << 8) | (all_hold_reg_data[(HREG_IDX_ON_SEC * 2) + 1]);}

      uint16_t get_internal_temp(){return (uint16_t) (all_hold_reg_data[HREG_IDX_TEMP * 2] << 8) | (all_hold_reg_data[(HREG_IDX_TEMP * 2) + 1]);}
      uint16_t get_external_temp(){return (uint16_t) (all_hold_reg_data[HREG_IDX_TEMP_EXD * 2] << 8) | (all_hold_reg_data[(HREG_IDX_TEMP_EXD * 2) + 1]);}

      bool get_lock_state(){return (bool) all_hold_reg_data[(HREG_IDX_LOCK * 2) + 1];}
      bool get_protect_state(){return (bool) all_hold_reg_data[(HREG_IDX_PROTECT * 2) + 1];}
      bool get_constant_state(){return (bool) all_hold_reg_data[(HREG_IDX_CVCC * 2) + 1];}
      bool get_switch_state(){return (bool) all_hold_reg_data[(HREG_IDX_OUTPUT_ON * 2) + 1];} 
      bool get_temp_symbol(){return (bool) all_hold_reg_data[(HREG_IDX_FC * 2) + 1];} 

      uint8_t get_model(){return all_hold_reg_data[(HREG_IDX_MODEL * 2) + 1];}
      uint8_t get_version(){return all_hold_reg_data[(HREG_IDX_VERSION * 2) + 1];}
      uint8_t get_address(){return all_hold_reg_data[(HREG_IDX_SLAVE_ADD * 2) + 1];}
      uint8_t get_baudrate(){return all_hold_reg_data[(HREG_IDX_BAUDRATE * 2) + 1];}

      uint16_t get_internal_temp_offset(){return (uint16_t) (all_hold_reg_data[HREG_IDX_TEMP_OFS * 2] << 8) | (all_hold_reg_data[(HREG_IDX_TEMP_OFS * 2) + 1]);}
      uint16_t get_external_temp_offset(){return (uint16_t) (all_hold_reg_data[HREG_IDX_TEMP_EXT_OFS * 2] << 8) | (all_hold_reg_data[(HREG_IDX_TEMP_EXT_OFS * 2) + 1]);}

      uint8_t get_loaded_preset(){return all_hold_reg_data[(HREG_IDX_MEMORY * 2) + 1];}

      bool fetch_preset(tMemory &presetStruct);

      //setter functions
      bool set_volt(uint16_t value) {return write_a_single_register(HREG_IDX_CV, value);}
      bool set_current(uint16_t value)  {return write_a_single_register(HREG_IDX_CC, value);}
      bool set_lock_state(bool value) {return write_a_single_register(HREG_IDX_LOCK, value);}  
      bool set_protect_state(bool value)  {return write_a_single_register(HREG_IDX_PROTECT, value);}
      bool set_switch_state(bool value) {return write_a_single_register(HREG_IDX_OUTPUT_ON, value);}
      bool set_temp_symbol(bool value)  {return write_a_single_register(HREG_IDX_FC, value);}
      bool set_sleep_time(uint16_t value) {return write_a_single_register(HREG_IDX_SLEEP, value);}
      bool set_address(uint16_t value)  {return write_a_single_register(HREG_IDX_SLAVE_ADD, value);}
      bool set_baudrate(uint16_t value) {return write_a_single_register(HREG_IDX_BAUDRATE, value);}
      bool set_internal_temp_offset(uint16_t value) {return write_a_single_register(HREG_IDX_TEMP_OFS, value);}
      bool set_external_temp_offset(uint16_t value) {return write_a_single_register(HREG_IDX_TEMP_EXT_OFS, value);}
      bool switch_preset(uint8_t value) {return write_a_single_register(HREG_IDX_MEMORY, value);}

      bool set_preset(tMemory &presetStruct);

    private:
      /**
       * @brief Calculates the crc16 checksum 
       * @param buf pointer to the data
       * @param length data length
       * 
       * @return crc16 value
       */
      uint16_t crc16_calculator(uint8_t *buf, uint8_t length);

      bool read_hold_register_data(uint16_t holding_reg_start_addr, uint16_t no_of_register_to_read);
      bool write_a_single_register(uint16_t reg_address, uint16_t value);
      bool write_multiple_registers(uint16_t holding_reg_start_addr, uint16_t no_of_register_to_write, uint8_t data_byte_count, uint8_t *value_buf);
      bool get_all_hold_regs();

    private:
      Stream *serialHandle;
      uint8_t _slave_address;
      uint8_t all_hold_reg_data[60];
      uint8_t response_temp_buf[70];
      uint16_t _timeout;
};

#endif