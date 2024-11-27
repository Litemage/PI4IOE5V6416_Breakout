#include <stdint.h>
#include "PI4IOE5V6416_defs.h"
#include "PI4IOE5V6416.h"

// ==================== INTERNAL API ====================

/**
 * \brief Interface & device check. Checks for null pointers and invalid address in a device object before use
 * 
 * \param pDev 
 * \return int PI4IO_OK on success
 * 
 * - PI4IO_ERR_NULL_PTR Either the read or write function pointer is null
 * - PI4IO_ERR_ADDR The device data structure is configured with an invalid address
 */
static int IntfCheck(const pi4io_dev_t *pDev);

// ==================== REGISTER INTERACTION API ====================

int
PI4IOGetInputPort(const pi4io_port_num_t portNum, uint8_t *pInputState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_INPUT_PORT_0 : PI4IO_REG_INPUT_PORT_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pInputState = buf;

   return PI4IO_OK;
}

int
PI4IOSetOutputPort(const pi4io_port_num_t portNum, const uint8_t outputState, const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_OUTPUT_PORT_0 : PI4IO_REG_OUTPUT_PORT_1, outputState);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

int
PI4IOGetOutputPort(const pi4io_port_num_t portNum, uint8_t *pOutputState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_OUTPUT_PORT_0 : PI4IO_REG_OUTPUT_PORT_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pOutputState = buf;

   return PI4IO_OK;
}

int
PI4IOSetPortInversePolarity(const pi4io_port_num_t portNum, const uint8_t invertedState, const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // REMOVEME: Testing
   /**
    * Either determine here, or in another function if a reset is required to set a pin as inverted
    * 
    * Should we just fail to the user? 
    * Should we reset and re-configure in the background?
    * 
    */
   // ! end

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_POLARITY_INV_0 : PI4IO_REG_POLARITY_INV_1, invertedState);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

int
PI4IOGetPortInversionPolarity(const pi4io_port_num_t portNum, uint8_t *pInvertedState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_POLARITY_INV_0 : PI4IO_REG_POLARITY_INV_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pInvertedState = buf;

   return PI4IO_OK;
}

int
PI4IOSetPinDirection(const pi4io_port_num_t portNum, const uint8_t directionState, const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_CONFIG_0 : PI4IO_REG_CONFIG_1, directionState);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

int
PI4IOGetPinDirection(const pi4io_port_num_t portNum, uint8_t *pDirectionState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_CONFIG_0 : PI4IO_REG_CONFIG_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pDirectionState = buf;

   return PI4IO_OK;
}

int
PI4IOGetOutputDriveStrength(const pi4io_port_num_t portNum, uint8_t (*pDrvStrength)[2], const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf[2] = {};

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_DRV_STREN_0_0 : PI4IO_REG_DRV_STREN_1_0, buf, 2);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   // Copy data to out buffer
   *pDrvStrength[0] = buf[0];
   *pDrvStrength[1] = buf[1];

   return PI4IO_OK;
}

int
PI4IOSetOuputDriveStrength(const pi4io_port_num_t portNum, const uint8_t (*pDrvStrength)[2], const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_DRV_STREN_0_0 : PI4IO_REG_DRV_STREN_0_0, *(pDrvStrength[0]));
   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_DRV_STREN_0_1 : PI4IO_REG_DRV_STREN_1_1, *(pDrvStrength[1]));

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   return PI4IO_OK;
}

int
PI4IOSetInputLatch(const pi4io_port_num_t portNum, const uint8_t latchState, const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_INPUT_LATCH_0 : PI4IO_REG_INPUT_LATCH_1, latchState);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

int
PI4IOGetInputLatch(const pi4io_port_num_t portNum, uint8_t *pLatchState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_INPUT_LATCH_0 : PI4IO_REG_INPUT_LATCH_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pLatchState = buf;

   return PI4IO_OK;
}

int
PI4IOSetPuPdEnable(const pi4io_port_num_t portNum, const uint8_t puPdState, const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_PU_PD_EN_0 : PI4IO_REG_PU_PD_EN_1, puPdState);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

int
PI4IOGetPuPdEnable(const pi4io_port_num_t portNum, uint8_t *pPuPdState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_PU_PD_EN_0 : PI4IO_REG_PU_PD_EN_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pPuPdState = buf;

   return PI4IO_OK;
}

int
PI4IOSetPUPD(const pi4io_port_num_t portNum, const uint8_t puPd, const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_PU_PD_CFG_0 : PI4IO_REG_PU_PD_CFG_1, puPd);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

int
PI4IOGetPUPD(const pi4io_port_num_t portNum, uint8_t *pPuPd, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_PU_PD_CFG_0 : PI4IO_REG_PU_PD_CFG_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pPuPd = buf;

   return PI4IO_OK;
}

int
PI4IOSetIntMask(const pi4io_port_num_t portNum, const uint8_t interruptState, const pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->write(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_INT_MASK_0 : PI4IO_REG_INT_MASK_1, interruptState);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

int
PI4IOGetIntMask(const pi4io_port_num_t portNum, uint8_t *pInterruptState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_INT_MASK_0 : PI4IO_REG_INT_MASK_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pInterruptState = buf;

   return PI4IO_OK;
}

int
PI4IOGetIntStatus(const pi4io_port_num_t portNum, uint8_t *pIntStatus, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, portNum == PI4IO_PORT_0 ? PI4IO_REG_INT_STAT_0 : PI4IO_REG_INT_STAT_1, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pIntStatus = buf;

   return PI4IO_OK;
}

int
PI4IOGetOutputPortConfig(uint8_t *pOutputCfg, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t buf;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = pDev->read(pDev->addr, PI4IO_REG_OUTPUT_PORT_CFG, &buf, 1);

   if (rslt != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Read OK

   *pOutputCfg = buf;

   return PI4IO_OK;
}

int
PI4IOSetOutputPortConfig(pi4io_port_num_t port, uint8_t portOutConfig, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t oldRegister;
   uint8_t newRegister;
   uint8_t portMask = 0x01 << (port == 0 ? 0 : 1);

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   if (pDev->read(pDev->addr, PI4IO_REG_OUTPUT_PORT_CFG, &oldRegister, 1) != 0)
   {
      return PI4IO_ERR_COMM;
   }

   if (portOutConfig != 0)
   {
      // Set bit
      newRegister = oldRegister |= portMask;
   }
   else
   {
      // Clear bit
      newRegister = oldRegister & ~(portMask);
   }

   if (pDev->write(pDev->addr, PI4IO_REG_OUTPUT_PORT_CFG, newRegister) != 0)
   {
      return PI4IO_ERR_COMM;
   }

   // Write OK

   return PI4IO_OK;
}

// ==================== DRIVER API ====================

int
PI4IOConfigPinDirection(const pi4io_port_num_t port,
                        const pi4io_pin_num_t pin,
                        const pi4io_pin_direction_t pinDir,
                        const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t rxBuf, txBuf = 0xFF; // Default register value
   const uint8_t mask = 1 << pin;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   if ((rslt = PI4IOGetPinDirection(port, &rxBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   switch (pinDir)
   {
      case PI4IO_PIN_IN:
         txBuf = (rxBuf | mask);
         break;
      case PI4IO_PIN_OUT:
         txBuf = (rxBuf & ~mask);
         break;
   }

   return PI4IOSetPinDirection(port, txBuf, pDev);
}

int
PI4IOArrayConfigPinDirection(const pi4io_port_num_t port,
                             const pi4io_pin_direction_t (*pPinDir)[8],
                             const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t mask;
   uint8_t txBuf = 0xFF; // Default state of register

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Create a txBuf with the array of pin directions
   for (size_t i = 0; i < 8; ++i)
   {
      mask = (1 << i);

      switch ((*pPinDir)[i])
      {
         case PI4IO_PIN_IN:
            // Set bit
            txBuf |= mask;
            break;
         case PI4IO_PIN_OUT:
            // Clear bit
            txBuf &= ~mask;
            break;
      }
   }

   // Send txBuf, return status

   return PI4IOSetPinDirection(port, txBuf, pDev);
}

// Note this will only affect pins configured as inputs
int
PI4IOConfigPolarity(const pi4io_port_num_t port,
                    const pi4io_pin_num_t pin,
                    const pi4io_pin_polarity_t polarity,
                    const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t rxBuf;
   uint8_t txBuf = 0; // Default register value
   const uint8_t mask = (1 << pin);

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   if ((rslt = PI4IOGetPortInversionPolarity(port, &rxBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   switch (polarity)
   {
      case PI4IO_PIN_NO_INVERT:
         // Clear bit to disable polarity inversion.
         txBuf = (rxBuf & ~mask);
         break;
      case PI4IO_PIN_INVERT:
         // Set bit to enable polarity inversion.
         txBuf = (rxBuf | mask);
         break;

      default:
         break;
   }

   return PI4IOSetPortInversePolarity(port, txBuf, pDev);
}

int
PI4IOArrayConfigPolarity(const pi4io_port_num_t port,
                         const pi4io_pin_polarity_t (*pPolarity)[8],
                         const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t mask;
   uint8_t txBuf = 0; // Default register value

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   for (size_t i = 0; i < 8; ++i)
   {
      mask = (1 << i);

      switch ((*pPolarity)[i])
      {
         case PI4IO_PIN_NO_INVERT:
            // Clear bit to disable polarity inversion.
            txBuf &= ~mask;
            break;
         case PI4IO_PIN_INVERT:
            // Set bit to enable polarity inversion.
            txBuf |= mask;
            break;

         default:
            break;
      }
   }

   return PI4IOSetPortInversePolarity(port, txBuf, pDev);
}

int
PI4IOConfigPuPd(const pi4io_port_num_t port,
                const pi4io_pin_num_t pin,
                const pi4io_pin_pu_pd_t puPd,
                const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t selRx, enRx;
   uint8_t selTx = 0xFF; // Default register value
   uint8_t enTx = 0;
   uint8_t mask = (1 << pin);

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   rslt = PI4IOGetPuPdEnable(port, &enRx, pDev);
   rslt = PI4IOGetPUPD(port, &selRx, pDev);

   if (rslt != PI4IO_OK)
   {
      return rslt;
   }

   switch (puPd)
   {
      case PI4IO_PULL_NONE:
         enTx = (enRx & ~mask);
         // State of sel doesn't matter
         break;
      case PI4IO_PU:
         enTx = (enRx | mask);
         selTx = (selRx | mask);
         break;
      case PI4IO_PD:
         enTx = (enRx | mask);
         selTx = (selRx & ~mask);
         break;
   }

   rslt = PI4IOSetPuPdEnable(port, enTx, pDev);

   if (enTx & mask)
   {
      rslt = PI4IOSetPUPD(port, selTx, pDev);
   }

   return rslt;
}

int
PI4IOArrayConfigPuPd(const pi4io_port_num_t port, const pi4io_pin_pu_pd_t (*pPuPd)[8], const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t mask;
   uint8_t selTx = 0xFF; // Default register value
   uint8_t enTx = 0;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   for (size_t i = 0; i < 8; ++i)
   {
      mask = (1 << i);
      switch ((*pPuPd)[i])
      {
         case PI4IO_PULL_NONE:
            enTx &= ~mask;
            // State of sel doesn't matter
            break;

         case PI4IO_PU:
            enTx |= mask;
            selTx |= mask;
            break;

         case PI4IO_PD:
            enTx |= mask;
            selTx &= ~mask;
            break;
      }
   }

   rslt = PI4IOSetPuPdEnable(port, enTx, pDev);
   rslt = PI4IOSetPUPD(port, selTx, pDev);

   return rslt;
}

// Designed for one-off configs.
int
PI4IOConfigPin(const pi4io_port_num_t port, const pi4io_pin_num_t pin, const pi4io_pin_cfg_t *pCfg, pi4io_dev_t *pDev)
{
   int rslt;

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Pin direction

   if ((rslt = PI4IOConfigPinDirection(port, pin, pCfg->pinDir, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Pin polarity inversion

   if ((rslt = PI4IOConfigPolarity(port, pin, pCfg->polarity, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Pull-up/Pull-down config and enable

   return PI4IOConfigPuPd(port, pin, pCfg->puPd, pDev);
}

// Meant for bulk config of each port.
int
PI4IOConfigPort(const pi4io_port_num_t port, const pi4io_port_cfg_t *pPortCfg, const pi4io_dev_t *pDev)
{
   int rslt;
   pi4io_pin_direction_t dirBuf[8] = {};
   pi4io_pin_polarity_t polarityBuf[8] = {};
   pi4io_pin_pu_pd_t puPdBuf[8] = {};

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Populate buffers
   for (size_t i = 0; i < 8; ++i)
   {
      dirBuf[i] = pPortCfg->pinConfig[i].pinDir;
      polarityBuf[i] = pPortCfg->pinConfig[i].polarity;
      puPdBuf[i] = pPortCfg->pinConfig[i].puPd;
   }

   // Port-wise output configuration (push-pull / open-drain)
   if ((rslt = PI4IOSetOutputPortConfig(port, pPortCfg->outputCfg, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Pin Direction

   if ((rslt = PI4IOArrayConfigPinDirection(port, &dirBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Pin polarity inversion
   if ((rslt = PI4IOArrayConfigPolarity(port, &polarityBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Pull-up/Pull-down
   return PI4IOArrayConfigPuPd(port, &puPdBuf, pDev);
}

int
PI4IOE5V6416Init(const uint8_t addr, pi4io_dev_t *pDev)
{
   if (addr != PI4IO_ADDR_PRM && addr != PI4IO_ADDR_SEC)
   {
      return PI4IO_ERR_ADDR;
   }

   pDev->addr = addr;

   // Check if interface pointers are null

   if (!pDev->read || !pDev->write)
   {
      return PI4IO_ERR_NULL_PTR;
   }

   // Reset device, but only if reset pointer is defined

   if (pDev->reset)
   {
      pDev->reset(pDev->addr, 0);
      // Take 30 nS to initiate reset, and 600 nS to actually reset, according to datasheet.
      pDev->delayUs(10);
      pDev->reset(pDev->addr, 1);
   }

   return PI4IO_OK;
}

int
PI4IOWriteOutput(const pi4io_port_num_t port, const pi4io_pin_num_t pin, const uint8_t state, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t rxBuf;
   uint8_t txBuf = 0;
   uint8_t mask = (1 << pin);

   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Check if pin is output, if not, return an invalid pin state error
   if ((rslt = PI4IOGetPinDirection(port, &rxBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // If not an output, return a config error
   if (rxBuf & mask)
   {
      // Input bit was set, writing to output register would do nothing
      return PI4IO_ERR_CONFIG;
   }

   // Get the current output buffer
   if ((rslt = PI4IOGetOutputPort(port, &txBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Output bit is set
   // Clear bit if state is zero, otherwise set bit
   txBuf = state == 0 ? txBuf & ~mask : txBuf | mask;

   return PI4IOSetOutputPort(port, txBuf, pDev);
}

int
PI4IOReadInput(const pi4io_port_num_t port, const pi4io_pin_num_t pin, uint8_t *pState, const pi4io_dev_t *pDev)
{
   int rslt;
   uint8_t rxBuf;
   uint8_t mask = (1 << pin);

   // Ensure device is initialized with interface functions
   if ((rslt = IntfCheck(pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   if ((rslt = PI4IOGetPinDirection(port, &rxBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   if (!(rxBuf & mask))
   {
      // Input bit wasn't set, input read would return garbage.
      return PI4IO_ERR_CONFIG;
   }

   // Good to read input

   if ((rslt = PI4IOGetInputPort(port, &rxBuf, pDev)) != PI4IO_OK)
   {
      return rslt;
   }

   // Input valid

   *pState = rxBuf & mask;

   return rslt;
}

// ==================== INTERNAL API ====================

static int
IntfCheck(const pi4io_dev_t *pDev)
{
   if (pDev)
   {
      if (pDev->read && pDev->write)
      {
         return PI4IO_OK;
      }
   }

   if (pDev->addr != PI4IO_ADDR_PRM && pDev->addr != PI4IO_ADDR_SEC)
   {
      return PI4IO_ERR_ADDR;
   }

   return PI4IO_ERR_NULL_PTR;
}