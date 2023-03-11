/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../core/macros.h"

#if ENABLED(CNC_COORDINATE_SYSTEMS)

#include "../../module/stepper.h"

#define DEBUG_M53

/**
 * Select a coordinate system and update the workspace offset.
 * System index -1 is used to specify machine-native.
 */
bool GcodeSuite::select_coordinate_system(const int8_t _new) {
  if (active_coordinate_system == _new) return false;     // no WCS system change just return
  active_coordinate_system = _new;
  xyz_float_t new_offset{0};
  if (WITHIN(_new, 0, MAX_COORDINATE_SYSTEMS - 1))
    new_offset = coordinate_system[_new];  // fetch index [_new] from WCS system array and put into new_offset[]
  LOOP_LINEAR_AXES(i) { // translates to -> for(i; i<3; i++)
    if (position_shift[i] != new_offset[i]) {
      position_shift[i] = new_offset[i];
      update_workspace_offset((AxisEnum)i); //TG set workspace_offset[] = home_offset[] + position_shift[] (aka new_offset[])
    }
  }
  return true;
}

/**
 * G53: Apply native workspace to the current move
 *
 * In CNC G-code G53 is a modifier.
 * It precedes a movement command (or other modifiers) on the same line.
 * This is the first command to use parser.chain() to make this possible.
 *
 * Marlin also uses G53 on a line by itself to go back to native space.
 */
void GcodeSuite::G53() {
  const int8_t old_system = active_coordinate_system;       // save active_ptr
  select_coordinate_system(-1); // Always remove workspace, sets workspace_offset[]=home_offset[]+new_offset[], -1 is native 
  #ifdef DEBUG_M53
    SERIAL_ECHOLNPGM("Go to native space");
    report_current_position();
  #endif

  if (parser.chain()) {           // Command to chain?
    process_parsed_command();     // ...process the chained command
    select_coordinate_system(old_system);
    #ifdef DEBUG_M53
      SERIAL_ECHOLNPGM("Go back to workspace ", old_system);  //Go back to workspace old_ptr (previously active_coordinate_system)
      report_current_position();
    #endif
  }
}

/**
 * G54-G59.3: Select a new workspace
 *
 * A workspace is an XYZ offset to the machine native space.
 * All workspaces default to 0,0,0 at start, or with EEPROM
 * support they may be restored from a previous session.
 *
 * G92 is used to set the current workspace's offset.
 */
void G54_59(uint8_t subcode=0) {
  const int8_t _space = parser.codenum - 54 + subcode;  // get decimal number after 'G', i.e. 54...59 for G54...G59, minus 54
  if (gcode.select_coordinate_system(_space)) {         // set workspace_offset[] = home_offset[] + new_offset[]
    SERIAL_ECHOLNPGM("Select workspace ", _space);
    report_current_position();
  }
}
void GcodeSuite::G54() { G54_59(); }
void GcodeSuite::G55() { G54_59(); }
void GcodeSuite::G56() { G54_59(); }
void GcodeSuite::G57() { G54_59(); }
void GcodeSuite::G58() { G54_59(); }
void GcodeSuite::G59() { G54_59(parser.subcode); }


//TG 9/29/22 - added new Gcode G39 here to report WCS table or current active Workspace
// G39 alone will output Active Workspace, G39 T will output table of all workspaces
void GcodeSuite::G39(){
  #define CUSTOM_AXES 4   // allows for "G5x" in front of X , Y , Z
  char Msg[48];
  if(parser.seen("T")){
    SERIAL_ECHOLNPGM("All Workspaces..... ");  //TG 9/29/22 added this line
    LOOP_INT_S_L_N(i, -1, MAX_COORDINATE_SYSTEMS){
      sprintf(Msg,"%s%d - %s","G",i+54, i == gcode.active_coordinate_system ? "*" : "  ");
      xyze_pos_t lpos = coordinate_system[i];
      SERIAL_ECHOPGM_P(LIST_N(DOUBLE(CUSTOM_AXES),
        Msg," " ,
        X_LBL, lpos.x,
        SP_Y_LBL, lpos.y,
        SP_Z_LBL, lpos.z,
        SP_I_LBL, lpos.i,
        SP_J_LBL, lpos.j,
        SP_K_LBL, lpos.k
      ), "\n"
      #if HAS_EXTRUDERS
        , SP_E_LBL, lpos.e
      #endif
      );
    }
  }
  else{  
    sprintf(Msg,"Active Workspace: %d: G%d %s", gcode.active_coordinate_system, gcode.active_coordinate_system+54,
            gcode.active_coordinate_system < 0 ? " (native WS)" : "" );  //TG 9/29/22 added this for G39 (no params)
    SERIAL_ECHOLN(Msg);
  }
 
}
#endif // CNC_COORDINATE_SYSTEMS
