/**
 * \file PI4IOE5V6416_defs.h
 * \authors Jacob Simeone (jsimeone0105@gmail.com)
 * \brief Macros for PI4IOE5V6416 driver
 * \date 2023-07-22
 * 
 */

#ifndef __PI4IOE5V6416_DEFS_H__
#define __PI4IOE5V6416_DEFS_H__

// ========== Errors ==========

#define PI4IO_ERR_BASE            (0)
#define PI4IO_OK                  (PI4IO_ERR_BASE - 0)
#define PI4IO_ERR_COMM            (PI4IO_ERR_BASE - 1) // Communication to hardware failed
#define PI4IO_ERR_NULL_PTR        (PI4IO_ERR_BASE - 2)
#define PI4IO_ERR_ARG             (PI4IO_ERR_BASE - 3) // Invalid argument
#define PI4IO_ERR_CONFIG          (PI4IO_ERR_BASE - 4) // Configuration error
#define PI4IO_ERR_ADDR            (PI4IO_ERR_BASE - 5) // Invalid address

// ========== Registers ==========

#define PI4IO_REG_INPUT_PORT_0    (0x00)
#define PI4IO_REG_INPUT_PORT_1    (0x01)
#define PI4IO_REG_OUTPUT_PORT_0   (0x02)
#define PI4IO_REG_OUTPUT_PORT_1   (0x03)
#define PI4IO_REG_POLARITY_INV_0  (0x04)
#define PI4IO_REG_POLARITY_INV_1  (0x05)
#define PI4IO_REG_CONFIG_0        (0x06)
#define PI4IO_REG_CONFIG_1        (0x07)
#define PI4IO_REG_DRV_STREN_0_0   (0x40)
#define PI4IO_REG_DRV_STREN_0_1   (0x41)
#define PI4IO_REG_DRV_STREN_1_0   (0x42)
#define PI4IO_REG_DRV_STREN_1_1   (0x43)
#define PI4IO_REG_INPUT_LATCH_0   (0x44)
#define PI4IO_REG_INPUT_LATCH_1   (0x45)
#define PI4IO_REG_PU_PD_EN_0      (0x46)
#define PI4IO_REG_PU_PD_EN_1      (0x47)
#define PI4IO_REG_PU_PD_CFG_0     (0x48)
#define PI4IO_REG_PU_PD_CFG_1     (0x49)
#define PI4IO_REG_INT_MASK_0      (0x4A)
#define PI4IO_REG_INT_MASK_1      (0x4B)
#define PI4IO_REG_INT_STAT_0      (0x4C)
#define PI4IO_REG_INT_STAT_1      (0x4D)
#define PI4IO_REG_OUTPUT_PORT_CFG (0x4F)

#define PI4IO_REG_MAX             (PI4IO_REG_OUTPUT_PORT_CFG)

/**
 * 
 * Signals to I2C read function to not specify the register address, resulting in
 * the driver reading from the "current" register, stored on the IC's register pointer
 * register. 
 * 
 * Only valid for reading
 * 
 */
#define PI4IO_REG_CURRENT         (0xFF)

// ========== ADDRESS ==========

#define PI4IO_ADDR_PRM            (0x20)
#define PI4IO_ADDR_SEC            (0x21)

// ========== Includes ==========

#include <stdint.h>
#include <stddef.h>

// ========== Type Aliases ==========

//TODO: Make these take in device handle as well

/** 
 * Implement I2C communication with this function
 * 
 * Reference datasheet for more information: https://www.mouser.com/datasheet/2/115/PI4IOE5V6416-1488866.pdf
 * 
 * The process for writing to the PI4IOE5V6416 is:
 * 
 * - Master puts start condition onto bus
 * - Master puts slave address onto bus, with the write bit
 * - Slave acknowledges 
 * - Master puts register address on bus
 * - Slave acknowledges
 * - Master puts register data onto the bus
 * - Slave acknowledges
 * - Master puts stop condition on the bus
 * 
 * Ensure to reference datasheet for further details
 * 
 * Return -1 on failure, and 0 on success
 * 
*/
typedef int (*pi4io_i2c_write_fptr_t)(const uint8_t reg, const uint8_t val);

/**
 * Implement I2C communication with this function
 * 
 * Reference datasheet for more information: https://www.mouser.com/datasheet/2/115/PI4IOE5V6416-1488866.pdf
 * 
 * The process for reading data from the PI4IOE5V6416 is:
 * 
 * - Master puts start condition on the bus
 * - Master puts slave address on the bus with write bit
 * - Slave acknowledges
 * - Master puts register address on bus
 * - Slave acknowledges
 * - Master puts start condition on bus
 * - Master puts slave address on the bus with read bit
 * - Slave acknowledges
 * - Slave submits K bytes of read data
 * - Master acknowledges, unless enough data has been received, in which case the master will not-acknowledge
 * - Master puts stop condition on bus
 * 
 * Ensure to reference datasheet for further details
 * 
 * Return -1 on failure, and 0 on success
 * 
 */
typedef int (*pi4io_i2c_read_fptr_t)(const uint8_t reg, uint8_t *pBuf, const size_t bufSize);

/**
 * \brief Pointer to a function that controls the reset pin. If null, user is responsible for 
 * resetting device. 
 * 
 * NOTE: set to null if reset pin is not used. Note this will stop the driver from resetting the 
 * device on every init call
 * 
 * \param pinState Represents the requested state of the reset pin. 0 for low, 1 for high.
 * 
 */
typedef int (*pi4io_reset_pin_fptr_t)(const uint8_t pinState);

/**
 * \brief Delay for x microseconds
 * 
 * \param delay Number of uS to delay.
 * 
 */
typedef void (*pi4io_delay_us_fptr_t)(const uint64_t delay);

// ========== Enums ==========

typedef enum
{
   PI4IO_PORT_0 = 0,
   PI4IO_PORT_1 = 1,

   PI4IO_PORT_MAX,
} pi4io_port_num_t;

typedef enum
{
   PI4IO_DRV_1 = 0, // PIN default, all drive current
   PI4IO_DRV_0_25,  // 0.25 of drive strength
   PI4IO_DRV_0_5,   // 0.5 of drive strength
   PI4IO_DRV_0_75,  // 0.75 of drive strength
} pi4io_drv_strn_t;

typedef enum
{
   PI4IO_OUT_PSHPLL = 0, // Push-Pull, pin default
   PI4IO_OUT_OPNDRN,     // Open-drain
} pi4io_output_cfg_t;

typedef enum
{
   PI4IO_PIN_IN = 0, // Pin default, input
   PI4IO_PIN_OUT,    // output
} pi4io_pin_direction_t;

typedef enum
{
   PI4IO_PIN_NO_LATCH = 0, // Pin default, non-latching input
   PI4IO_PIN_LATCH,        // Latching intpu
} pi4io_latching_t;

typedef enum
{
   PI4IO_PIN_NO_INVERT = 0,
   PI4IO_PIN_INVERT,
} pi4io_pin_polarity_t;

typedef enum
{
   PI4IO_PULL_NONE = 0, // Default, No pull-up, or pull down.
   PI4IO_PU,            // pull-up resistor
   PI4IO_PD,            // Pull-down resistor
} pi4io_pin_pu_pd_t;

typedef enum
{
   PI4IO_PIN_0 = 0,
   PI4IO_PIN_1,
   PI4IO_PIN_2,
   PI4IO_PIN_3,
   PI4IO_PIN_4,
   PI4IO_PIN_5,
   PI4IO_PIN_6,
   PI4IO_PIN_7,

   PI4IO_PIN_MAX,
} pi4io_pin_num_t;

// ========== Structs ==========

typedef struct
{
   pi4io_pin_direction_t pinDir;
   pi4io_pin_pu_pd_t puPd;         // Ignored if pin is not an input
   pi4io_drv_strn_t outputDrvStrn; // Drive strength, only valid for pins configured as output
   pi4io_latching_t pinLatch;      // Only applies to inputs
   pi4io_pin_polarity_t polarity;  // Polarity is inverted. Only active for inputs
   uint8_t willInterrupt;          // If not zero, will configure this channel's mask to trigger interrupts
} pi4io_pin_cfg_t;

typedef struct
{
   pi4io_output_cfg_t outputCfg; // push-pull by default
   pi4io_pin_cfg_t pinConfig[8];
} pi4io_port_cfg_t;

typedef struct
{
   pi4io_i2c_write_fptr_t write;  // I2C Write callback
   pi4io_i2c_read_fptr_t read;    // I2C Read Callback
   pi4io_reset_pin_fptr_t reset;  // Reset gpio function
   pi4io_delay_us_fptr_t delayUs; // Delay for some microseconds
   uint8_t addr;
} pi4io_dev_t;

#endif // __PI4IOE5V6416_DEFS_H__