/** //TG MODIFIED BY T.GIOIOSA  - adds M8000 code for Filament Width Sensor calibration
 * original 8/27/23
 * updated  8/27/23
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * M codes for data exchange between Marlin and TFT35 screen over UART
 * The direction is from the TFT point of view
 * 
 * The main `loop()` gets the command at the front the G-code queue (if any) and runs it. Each G-code command blocks the main loop, preventing the queue
 * from advancing until it returns. To keep essential tasks and the UI running, any commands that run a long process need to call `idle()` frequently.
 * Don't use idle(); in loops waiting on I2C comm, it's not necessary and may actually hurt I2C comm. Use a NOP to insure while() allows CPU some time.
 * REMEMBER - Whenever the G-code processor is busy processing a command, the G-code queue cannot advance.
 */

#include <src/MarlinCore.h>
#include "../gcode.h"
#include "../../inc/MarlinConfig.h"
#include "M7800.h"
#include "../../feature/filwidth.h"
#include "../../feature/host_actions.h"

void GcodeSuite::M7800()    //TG 8/27/23 new!
{
  if (parser.seenval('S'))
  {
    
    FilamentWidthSensor::N_FW = parser.value_int();
    SERIAL_ECHOLNPGM("M7800 ok");                         // acknowledge receipt
  }


}