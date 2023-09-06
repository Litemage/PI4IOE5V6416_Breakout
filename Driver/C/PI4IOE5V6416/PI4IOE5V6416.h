/**
 * \file PI4IOE5V6416.h
 * \authors Jacob Simeone (jsimeone0105@gmail.com)
 * \brief PI4IOE5V6416 Driver
 * \date 2023-07-24
 * 
 * TODO PARKING
 * 
 * - [X] Remove interface file, replace with device handle
 * - [X] Combine Low-Level module with this
 * - [X] Switch all functions to take a device handle to get communication functions
 * - [X] Init function that resets device, and initializes device handle
 * - [X] Pin direction function. (array and single)
 * - [X] Polarity inversion function. (array and single)
 * - [X] Pull-up/Pull-down configuration & enable
 * - [X] Config function that will configure one "port" at a time
 * - [X] Output Write function
 * - [X] Input Read function
 * - [X] Null pointer check on configuration functions that depend on dev pointer
 * - [X] All TX buffers are initialized with default register value
 * - [X] Make "PI4" and "PI4IO" naming consistent
 * - [ ] Integrate and test with either hardware or breakout board
 * - [ ] Confirm latching behavior, the wording in the datasheet is odd...
 * 
 * Nice to have:
 * - [ ] Interrupt support
 * - [ ] Input latching support
 * - [ ] Error checking for valid arguments in driver API functions
 * - [ ] Split into multiple layers? Files are getting kinda long
 * - [ ] Add interrupt mask config functions
 * - [ ] Add latching config functions
 * 
 */

#ifndef __PI4IOE5V6416_H__
#define __PI4IOE5V6416_H__

#include "PI4IOE5V6416_defs.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

   // ==================== REGISTER INTERACTION API ====================

   /**
   * \brief Read the state of input pins. Will read regardless if configured as an input or output.
   * 
   * \param portNum Port number to read from.
   * \param [out] pInputState pInputState represents the state of pins on the IO expander,
    *\param pDev Initialized device pointer.
   *  with the value of pin portNum.x being (inputState >> x) & 0x01
   * 
   * \return 0 on success, otherwise an error code
   * 
   * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed
   */
   int PI4IOGetInputPort(const pi4io_port_num_t portNum, uint8_t *pInputState, const pi4io_dev_t *pDev);

   /**
   * \brief Set the output state of a given port, with each port pin encoded in LSB first, starting with pin 0, and increasing from there.
   * 
   * For example, for port 0 or 1:
   *  0101 0011
   *  |||| |||^ Pin 0
   *  |||| ||^ Pin 1
   *  |||| |^ Pin 2
   *  |||| ^ Pin 3
   *  |||^ Pin 4
   *  ||^ Pin 5
   *  |^ Pin 6
   *  ^ Pin 7
   * 
   * Setting a bit causes a logical high output, clearing results in a logical low.
   * 
   * Note this will have no effect on pins configured as inputs.
   * 
   * \param portNum Port number to set output state of
   * \param outputState The output state of a port's pins, as shown above
    *\param pDev Initialized device pointer.
   * \return int 0 on success
   * 
   * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed
   */
   int PI4IOSetOutputPort(const pi4io_port_num_t portNum, const uint8_t outputState, const pi4io_dev_t *pDev);

   /**
   * \brief Read the current states of outputs on a given port. Data is returned in the same format that it's 
   * written in, see PI4IOSetOutputPort for description. 
   * 
   * Note that this will return the value written to the register, and not the state of the pin. If configured
   * as an input, the value for that pin should be considered invalid.
   * 
   * \param portNum Port number to get output state of.
   * \param [out] pOutputState Buffer for the output state of the given port.
    *\param pDev Initialized device pointer.
   * \return int 0 on success.
   * 
   * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed
   */
   int PI4IOGetOutputPort(const pi4io_port_num_t portNum, uint8_t *pOutputState, const pi4io_dev_t *pDev);

   /**
   * \brief Configure reverse polarity for pins on a port. Note this is only valid for input pins
   * 
   * Note that if a pin is configured with inverse polarity, and then the corresponding bit is cleared the polarity is retained until reset.
   * 
   * \param portNum Port number to configure inverse polarity of.
   * \param invertedState Buffer for inverting pins. A bit of 1 indicates a pin is inverted. See PI4IOSetOutputPort for details.
    *\param pDev Initialized device pointer.
   * \return int 0 on success
   * 
   * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed
   */
   int
   PI4IOSetPortInversePolarity(const pi4io_port_num_t portNum, const uint8_t invertedState, const pi4io_dev_t *pDev);

   /**
    * \brief Get port's pins polarity. Note this is only valid for input pins.
    * 
    * \param portNum Port number to read from.
    * \param pInvertedState State of polarity config register.
    * \param pDev Initialized device pointer.
    * \return int 
    */
   int PI4IOGetPortInversionPolarity(const pi4io_port_num_t portNum, uint8_t *pInvertedState, const pi4io_dev_t *pDev);

   /**
    * \brief Configures the direction of I/O pins on the expander. 
    * 
    * A 1 in a pins bit position in the corresponding port's register indicates a high-impedance input
    * If the bit is cleared to 0, that pin is configured as an output. 
    * 
    * Note that on startup, both ports' pins default to input (1)
    * 
    * \param portNum Port number to configure
    * \param directionState Buffer representing the direction of each pin for a given port. See PI4IOSetOutputPort for details
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOSetPinDirection(const pi4io_port_num_t portNum, const uint8_t directionState, const pi4io_dev_t *pDev);

   /**
    * \brief Read the direction of a port's I/O pins
    * 
    * \param portNum Port number to read from
    * \param directionState Buffer representing the direction for each pin on a given port. See PI4IOSetOutputPort for details
    * \param pDev Initialized device pointer.
    * \return int 
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetPinDirection(const pi4io_port_num_t portNum, uint8_t *pDirectionState, const pi4io_dev_t *pDev);

   /**
    * \brief Read the drive strength of a pin on a given port
    * 
    * \param portNum Port number to read from
    * \param drvStrength An array of 2 bytes, with the first one being the state of pins 7-4, and the second 3-0.
    * \param pDev Initialized device pointer.
    * Each 8-byte value stores 4 crumbs that describe the output drive strength of a given port's pin.
    * 
    * See PI4IOSetOutputDriveStrength for an example of the data format.
    * 
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetOutputDriveStrength(const pi4io_port_num_t portNum, uint8_t (*pDrvStrength)[2], const pi4io_dev_t *pDev);

   /**
    * \brief Set the output drive strength of a pin on a given port.
    * 
    * Each pin has two bits for drive strength:
    * 0b00 = 0.25 of full drive strength
    * 0b01 = 0.50 of full drive strength
    * 0b10 = 0.75 of full drive strength
    * 0b11 = 1.00 of full drive strength
    * 
    * \param portNum Port number to configure
    * \param drvStrength An array of 2 bytes, with the first one being the state of pins 7-4, and the second 3-0.
    * Each 8-byte value stores 4 crumbs that describe the output drive strength of a given port's pin.
    * 
    * For example, to configure pin 0 with half drive strength, pin 2 with full drive strength, and pin 6 with 0.75 drive strength,
    * this would be the value of the array of 2 bytes:
    * 
    *     byte 0           byte 1
    * [ 00 10 00 00 ] [ 00 11 00 01 ]
    * 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOSetOuputDriveStrength(const pi4io_port_num_t portNum,
                                  const uint8_t (*pDrvStrength)[2],
                                  const pi4io_dev_t *pDev);

   /**
    * \brief Sets a port's pins latching mode. Note this only effects pins which are configured as inputs. 
    * 
    * See c.vi in the datasheet for more information on latching behavior
    * 
    * \param portNum Port number to configure
    * \param latchState A buffer representing the latch state of each pin for a given port. See PI4IOSetOutputPort for 
    * more information on buffer format.
    * 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOSetInputLatch(const pi4io_port_num_t portNum, const uint8_t latchState, const pi4io_dev_t *pDev);

   /**
    * \brief Gets a port's pins latching mode. Note this only effects pins which are configured as inputs. 
    * 
    * See c.vi in the datasheet for more information on latching behavior
    * 
    * \param portNum Port number to read configuration of
    * \param pLatchState A buffer to store the latch state of each pin for a given port. See PI4IOSetOutputPort for 
    * more information on buffer format
    * 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetInputLatch(const pi4io_port_num_t portNum, uint8_t *pLatchState, const pi4io_dev_t *pDev);

   /**
    * \brief Enable Pull-up/Pull-down resistors on a port's pins
    * 
    * \param portNum Port number to configure
    * \param puPdState Buffer representing the PU/PD enable state of each pin for a given port. See PI4IOSetOutputPort
    * for more information on buffer format.
    * 1 indicates pull-up/pull-down resistors are enabled
    * 0 indicates disabled
    * 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOSetPuPdEnable(const pi4io_port_num_t portNum, const uint8_t puPdState, const pi4io_dev_t *pDev);

   /**
    * \brief Get a port's pins Pull-up/Pull-down enable status
    * 
    * \param portNum Port to read from
    * \param puPdState Buffer representing the PU/PD enable state of each pin for a given port. See PI4IOSetOutputPort
    * for more information on buffer format.
    * 1 indicates pull-up/pull-down resistors are enabled
    * 0 indicates disabled
    * 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetPuPdEnable(const pi4io_port_num_t portNum, uint8_t *pPuPdState, const pi4io_dev_t *pDev);

   /**
    * \brief Set a port's pins to use Pull-up or Pull-down resistors. 
    * 1 = pull up
    * 0 = pull down
    * 
    * Note this will only work if Pull-up/Pull-down has been enabled on a port's pin(s)
    * 
    * \param portNum Port number to configure
    * \param puPd Buffer representing a Pull-up/Pull-down resistor configuration on each of a port's pins. See PI4IOSetOutputPort
    * for more information on buffer format
    * 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOSetPUPD(const pi4io_port_num_t portNum, const uint8_t puPd, const pi4io_dev_t *pDev);

   /**
    * \brief Get a port's pins Pull-up/Pull-down configuration
    * 1 = pull up
    * 0 = pull down
    * 
    * Note this will only work if Pull-up/Pull-down has been enabled on a port's pin(s)
    * 
    * \param portNum Port number to read PU/PD config from
    * \param pPuPd Buffer to store current PU/PD config or selected port. 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetPUPD(const pi4io_port_num_t portNum, uint8_t *pPuPd, const pi4io_dev_t *pDev);

   /**
    * \brief Set a port's pins interrupts. On startup, all interrupts are disabled with all "1's" in the registers.
    * 
    * 0 = Interrupt enabled
    * 1 = Interrupt disabled
    * 
    * Reference datasheet for more information;
    * 
    * \param portNum Port number to configure
    * \param interruptState Buffer representing a mask for interrupts, see PI4IOSetOutputPort for more information
    * on buffer format.
    * 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOSetIntMask(const pi4io_port_num_t portNum, const uint8_t interruptState, const pi4io_dev_t *pDev);

   /**
    * \brief Get a prot's interrupt mask configuration.
    * 
    * \param portNum Port number to read interrupt config from.
    * \param pInterruptState Buffer to store interrupt state in.
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetIntMask(const pi4io_port_num_t portNum, uint8_t *pInterruptState, const pi4io_dev_t *pDev);

   /**
    * \brief Get interrupt status from read-only interrupt register. Use this to identify the source of an interrupts. 
    * 
    * 0 -> Pin was NOT the source of an interrupt
    * 1 -> Pin was the source of an interrupt.
    * 
    * Note that after reading, the interrupt status bit will be cleared to a logical 0
    * 
    * \param portNum 
    * \param pIntStatus 
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetIntStatus(const pi4io_port_num_t portNum, uint8_t *pIntStatus, const pi4io_dev_t *pDev);

   /**
    * \brief Set the output configuration of a given port to either push-pull or open-drain
    * 
    * \param port Port number to set output mode of
    * \param portOutConfig 0 for push-pull (default) or 1 for open-drain
    * \param pDev Initialized device pointer
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    * - PI4_ERR_ADDR -> Device pointer has improperly initialized address
    * - PI4_ERR_NULL_PTR -> Device pointer has a null pointer
    */
   int PI4IOSetOutputPortConfig(pi4io_port_num_t port, uint8_t portOutConfig, const pi4io_dev_t *pDev);

   /**
    * \brief Get the output configuration of a given port. 
    * 
    * \param pOutputCfg Buffer to store current output config register value in.
    * \param pDev Initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication between master and PI4IOE5V6416 failed.
    */
   int PI4IOGetOutputPortConfig(uint8_t *pOutputCfg, const pi4io_dev_t *pDev);

   // ==================== DRIVER API ====================

   /**
    * \brief Configures the direction of a single pin on a given port. Performs a read/write exchange for every call.
    * 
    * Note: Intended for one-off pin configuration, use array variant of this function to bulk configure ports.
    * 
    * \param port I/O port's pin to configure
    * \param pin Pin of the I/O port to set direction of
    * \param pinDir Direction of the I/O pin
    * \param pDev An initialized device pointer.
    * \return int 0 on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOConfigPinDirection(const pi4io_port_num_t port,
                               const pi4io_pin_num_t pin,
                               const pi4io_pin_direction_t pinDir,
                               const pi4io_dev_t *pDev);

   /**
    * \brief Bulk configure direction of all the pins on a port. Performs one read/write exchange per call.
    * 
    * Note: Intended for bulk pin configuration, see PI4IOConfigPinDirection for single pin configuration.
    * 
    * \param port Port's pins to configure.
    * \param pPinDir I/O direction of each pin, index corresponds to pin on port, i.e. (*pPinDir)[0] is the value of pin X.0,
    * where X is the port number.
    * 
    * \param pDev An initialized device pointer
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOArrayConfigPinDirection(const pi4io_port_num_t port,
                                    const pi4io_pin_direction_t (*pPinDir)[8],
                                    const pi4io_dev_t *pDev);

   /**
    * \brief Configures the polarity of a single pin on a given port. Performs a read/write exchange for every call.
    * 
    * Note: Intended for one-off pin configuration, use array variant of this function to bulk configure ports.
    * 
    * Note: This function will only effect pins that are configured as inputs.
    * 
    * \param port Port's pins to configure.
    * \param pin Pin of the I/O port to set polarity of.
    * \param polarity Indicates if a pin's input is inverted.
    * \param pDev An initialized device pointer
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOConfigPolarity(const pi4io_port_num_t port,
                           const pi4io_pin_num_t pin,
                           const pi4io_pin_polarity_t polarity,
                           const pi4io_dev_t *pDev);

   /**
    * \brief Bulk configure polarity of all the pins on a port. Performs one read/write exchange per call.
    * 
    * Note: Intended for bulk pin configuration, see PI4IOConfigPolarity for single pin configuration.
    * 
    * Note: This function will only effect pins that are configured as inputs.
    * 
    * \param port Port's pins to configure.
    * \param pPolarity Polarity of each pin, index corresponds to pin on port, i.e. (*pPolarity)[0] is the value of pin X.0,
    * where X is the port number
    * 
    * \param pDev An initialized device pointer
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOArrayConfigPolarity(const pi4io_port_num_t port,
                                const pi4io_pin_polarity_t (*pPolarity)[8],
                                const pi4io_dev_t *pDev);

   /**
    * \brief Configures Pull-up/Pull-down of a single output pin on a given port. Performs a read/write exchange for every call.
    * 
    * Note: Intended for one-off pin configuration, use array variant of this function to bulk configure ports.
    * 
    * Note: This function will only effect pins that are configured as outputs.
    * 
    * \param port Port's pins to configure.
    * \param pin Pin of the I/O port to set pull-up/pull-down config of.
    * \param polarity The PU/PD type.
    * \param pDev An initialized device pointer.
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOConfigPuPd(const pi4io_port_num_t port,
                       const pi4io_pin_num_t pin,
                       const pi4io_pin_pu_pd_t puPd,
                       const pi4io_dev_t *pDev);

   /**
    * \brief Bulk configure Pull-up/Pull-down of output pins on a given port. Performs a single read/write exchange for every call.
    * 
    * Note: Intended for bulk pin configuration, see PI4IOConfigPuPd for single pin configuration.
    * 
    * Note: This function will only effect pins that are configured as outputs.
    * 
    * \param port Port's pins to configure.
    * \param pPuPd Pull-up/Pull-down configuration of each pin. Index corresponds to pin on port, 
    * i.e. (*pPolarity)[0] is the value of pin X.0, where X is the port number.
    * 
    * \param pDev An initialized device pointer
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOArrayConfigPuPd(const pi4io_port_num_t port, const pi4io_pin_pu_pd_t (*pPuPd)[8], const pi4io_dev_t *pDev);

   /**
    * \brief Configure a single pin on a given port. Use this function when only configuring a couple pins.
    * 
    * \param port pin's port.
    * \param pin Pin on port "port" to configure.
    * \param pCfg Configuration structure for a given pin
    * \param pDev An initialized device pointer.
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOConfigPin(const pi4io_port_num_t port,
                      const pi4io_pin_num_t pin,
                      const pi4io_pin_cfg_t *pCfg,
                      pi4io_dev_t *pDev);

   /**
    * \brief Bulk configure a port. Use this function when initializing whole ports.
    * 
    * \param port Port to configure.
    * \param pPortCfg Port configuration structure, which contains 8 pin config structs.
    * \param pDev An initialized device pointer.
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOConfigPort(const pi4io_port_num_t port, const pi4io_port_cfg_t *pPortCfg, const pi4io_dev_t *pDev);

   /**
    * \brief Initialize a new PI4IOE5V6416 IO expander. Ensure to assign callbacks to write and read functions. This 
    * function also assigns this particular device's address.
    * 
    * \param dev Pointer to a pi4io device data structure
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOE5V6416Init(const uint8_t addr, pi4io_dev_t *pDev);

   /**
    * \brief Write output state to a given pin
    * 
    * \param port Port of pin
    * \param pin Pin on port to set output of
    * \param state 0 for "off", 1 for "on"
    * \param pDev Initialized device pointer
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOWriteOutput(const pi4io_port_num_t port,
                        const pi4io_pin_num_t pin,
                        const uint8_t state,
                        const pi4io_dev_t *pDev);

   /**
    * \brief Read the state of a given input pin
    * 
    * \param port Port of pin
    * \param pin Pin on port to read input of
    * \param [out] pState Pointer to a buffer to hold the state of the pin
    * \param pDev Initialized device pointer
    * \return int PI4_OK on success
    * 
    * - PI4_ERR_COMM -> Communication with hardware failed
    * - PI4_ERR_NULL_PTR -> Device pointer is not populated with read/write function pointers
    * - PI4_ERR_ADDR -> Address stored in device pointer is not valid
    */
   int PI4IOReadInput(const pi4io_port_num_t port, const pi4io_pin_num_t pin, uint8_t *pState, const pi4io_dev_t *pDev);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __PI4IOE5V6416_H__