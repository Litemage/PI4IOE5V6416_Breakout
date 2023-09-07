/**
 *
 * ========================= LICENSE =============================
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ===============================================================
 *
 * @file PI4IO_Example.c
 * @author Jacob Simeone (jsimeone0105@gmail.com)
 *
 * @brief An example demonstrating the use of the PI4IOE5V6416 C driver included
 * in the breakout board repository found at (as of date on this file):
 *
 * https://github.com/Litemage/PI4IOE5V6416_Breakout
 *
 * You will need to implement some functions specific to your platform first,
 * before the demo can work.
 *
 * To use the demo, you can then either copy/paste the code in the
 * "main" function to your own "main" (along with the other contents of this
 * file) or change the name to something other than "main", and call this
 * example's "main" from your code.
 *
 * Please note that this example's "main" is blocking.
 *
 * The data sheet for the PI4IOE5V6416 can be found here (as of date on this
 * file):
 *
 * https://www.diodes.com/assets/Datasheets/PI4IOE5V6416.pdf
 *
 * @version 1.0
 * @date 2023-09-06
 *
 */

#include "PI4IOE5V6416.h"
#include "PI4IOE5V6416_defs.h"
#include <stdint.h>

// ==================== STATIC VARIABLES ====================
/**
 * These are statically-allocated variables that are needed with every public
 * API call, initialize / store them however you want, but statically allocating
 * them at the global scope isn't a bad idea.
 */

static pi4io_dev_t pi4io = {};

/**
 * This is an example of a configuration structure that is used to configure
 * the I/O expander, by each port.
 */

// PORT 0 ON EXPANDER
pi4io_port_cfg_t port0Conf = {
    .outputCfg = PI4IO_OUT_OPNDRN, // Outputs are open-drain.
    .pinConfig =
        {
            [0] =
                {
                    .pinDir = PI4IO_PIN_OUT, // Configure direction as output
                },
            [1] =
                {
                    .pinDir = PI4IO_PIN_OUT, // Configure direction as an output
                },
            [2] =
                {
                    .pinDir = PI4IO_PIN_IN, // Configure pin as an input
                    .puPd = PI4IO_PD,       // Configure this pin to pull down
                },
            /* This pin can be thought of as configured as an active low input,
               like the rest of the pins */
            [3] =
                {
                    .pinDir = PI4IO_PIN_IN,       // Input
                    .polarity = PI4IO_PIN_INVERT, // Inverted state in input
                                                  //  register. Logical high on
                                                  //  pin = 0. Logical low = 1
                    .puPd = PI4IO_PU,             // Pull-up this pin
                },
            [4] =
                {
                    .pinDir = PI4IO_PIN_IN,
                    .polarity = PI4IO_PIN_INVERT,
                    .puPd = PI4IO_PU,
                },
            [5] =
                {
                    .pinDir = PI4IO_PIN_IN,
                    .polarity = PI4IO_PIN_INVERT,
                    .puPd = PI4IO_PU,
                },
            [6] =
                {
                    .pinDir = PI4IO_PIN_IN,
                    .polarity = PI4IO_PIN_INVERT,
                    .puPd = PI4IO_PU,
                },
            [7] =
                {
                    .pinDir = PI4IO_PIN_IN,
                    .polarity = PI4IO_PIN_INVERT,
                    .puPd = PI4IO_PU,
                },

        },
};

// ==================== CALLBACK FUNCTIONS ====================
/**
 * These functions need to be defined per-implementation, and describe how to
 * read & write over i2c, how to wait, and how to toggle the reset pin on the
 * I/O expander.
 *
 * Details for the I2C reading & writing interactions can be found on the
 * data sheet.
 */

static int pi4io_i2c_write(const uint8_t reg, const uint8_t val) {
  /**
   * TODO Implement: Write to a register
   */
}

static int pi4io_i2c_read(const uint8_t reg, uint8_t *pBuf,
                          const size_t bufSize) {
  /**
   * TODO Implement: Read from a register
   */
}

static int pi4io_set_reset_pin(const uint8_t pinState) {
  /**
   * TODO Implement: Set reset pin to given pinState
   */
}

static void pi4io_delay_uS(const uint8_t delay) {
  /**
   * TODO Implement: delay some number of uS
   */
}

// ==================== EXAMPLE-SPECIFIC FUNCTIONS ====================

/**
 * @brief Delay a given number of mS (used for this example's "main" function)
 *
 * @param mS
 */
delay_mS(uint32_t mS) {
  /**
   * TODO Implement: delay some number of mS
   */
}

// ==================== INITIALIZATION ====================

/**
 * @brief A demonstration on how you might go about initializing the I/O
 * expander
 *
 * @return int The result of the various initialization & configuration API
 * calls
 */
static int pi4io_init() {

  int rslt = -1;
  pi4io.write = pi4io_i2c_write;
  pi4io.read = pi4io_i2c_read;
  pi4io.reset = pi4io_set_reset_pin;
  pi4io.delayUs = pi4io_delay_uS;

  /**
   * NOTE: If you don't assign the above functions to the device handle before
   * hand, the Init function will return an error
   */
  if ((rslt = PI4IOE5V6416Init(PI4IO_ADDR_PRM, &pi4io)) != 0) {
    return rslt;
  }

  /**
   * Use this function to configure by port, with the structure as initialized
   * above
   */
  if ((rslt = PI4IOConfigPort(PI4IO_PORT_0, &port0Conf, &pi4io)) != PI4IO_OK) {
    return rslt;
  }

  /** NOTE:
   * Not initializing port 1 causes it to be configured with initial register
   * values as described in the data sheet
   */
}

// ==================== MAIN PROGRAM ====================

/**
 * This is an example main function, you would replace the contents of
 * your main function with the statements in the function "main"'s body.
 *
 * Otherwise, rename this function and call it from your code.
 *
 * Connecting an LED (or another appropriate load) to pin 0 of port 0 will
 * result in a blink with a period of 2 seconds.
 */
int main() {
  int rslt = -1;
  rslt = pi4io_init();

  for (;;) {
    PI4IOWriteOutput(0, 0, 1, &pi4io);
    delay_mS(1000);
    PI4IOWriteOutput(0, 0, 0, &pi4io);
    delay_mS(1000);
  }
}