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

#include "../../inc/MarlinConfig.h"

#if HAS_BED_PROBE

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/probe.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../lcd/marlinui.h"

#if HAS_PTC
  #include "../../feature/probe_temp_comp.h"
#endif

#if HAS_MULTI_HOTEND
  #include "../../module/tool_change.h"
#endif

/**
 * G30: Do a single Z probe at the given XY (default: current)
 *
 * Parameters:
 *
 *   X   Probe X position (default current X)
 *   Y   Probe Y position (default current Y)
 *   E   Engage the probe for each probe (default 1)
 *   C   Enable probe temperature compensation (0 or 1, default 1)
 */
void GcodeSuite::G30() {

  //xy_pos_t old_pos = current_position;  //TG 5/15/23 not needed cause removed returning to oldpos below 
  xy_pos_t probepos = current_position; //TG start with current position as default
report_current_position(); 
  //TG  If X,Y supplied, convert given logical position to native position (override default)
  const bool seenX = parser.seenval('X');
  if (seenX) probepos.x = RAW_X_POSITION(parser.value_linear_units());
  const bool seenY = parser.seenval('Y');
  if (seenY) probepos.y = RAW_Y_POSITION(parser.value_linear_units());

  probe.use_probing_tool(); //TG call to avoid tool-changes if probing multiple points

    // Convert the given logical position to native position
  const xy_pos_t pos = {
    parser.seenval('X') ? RAW_X_POSITION(parser.value_linear_units()) : current_position.x,
    parser.seenval('Y') ? RAW_Y_POSITION(parser.value_linear_units()) : current_position.y
  };

  if (probe.can_reach(probepos)) {
    //TG 5/13/23 **** with these 2 linescommented out the "set probe offset" behaves like
    // it used to, instead of putting nozzle at 2X the probe offset(51,-39) when ready
    // although the "level corners" run now returns to center after each corner is done
    // instead of jumping back to corner pos - probe offset(51,-39) for each corner
    // NEED TO RESOLVE THIS OR MAYBE MODIFY TFT35 CODE TO FIX ????????????
    //if (seenX) old_pos.x = probepos.x;
    //if (seenY) old_pos.y = probepos.y;
  
    // Disable leveling so the planner won't mess with us
    TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));
    //TG temporarily remove any feedrate multiplier
    remember_feedrate_scaling_off();

    TERN_(DWIN_CREALITY_LCD_JYERSUI, process_subcommands_now(F("G28O")));
    //TG stow probe after or not?
    const ProbePtRaise raise_after = parser.boolval('E', true) ? PROBE_PT_STOW : PROBE_PT_NONE;
    //TG enable probe temperature compensation?
    TERN_(HAS_PTC, ptc.set_enabled(!parser.seen('C') || parser.value_bool()));
    
    //TG do a Z-probe at the current position
    const float measured_z = probe.probe_at_point(probepos, raise_after, 1);
    TERN_(HAS_PTC, ptc.set_enabled(true));
    
    if (!isnan(measured_z)) {  //TG if successful report coordinates now
      SERIAL_ECHOLNPGM("Bed X: ", probepos.asLogical().x, " Y: ", probepos.asLogical().y, " Z: ", measured_z);
      #if EITHER(DWIN_LCD_PROUI, DWIN_CREALITY_LCD_JYERSUI)
        char msg[31], str_1[6], str_2[6], str_3[6];
        sprintf_P(msg, PSTR("X:%s, Y:%s, Z:%s"),
          dtostrf(probepos.x, 1, 1, str_1),
          dtostrf(probepos.y, 1, 1, str_2),
          dtostrf(measured_z, 1, 2, str_3)
        );
        ui.set_status(msg);
      #endif
    }

    restore_feedrate_and_scaling(); //TG restore any feedrate multiplier

    //TG 5/12/23 ***** THIS IS ACTING WEIRD WHY IS IT HERE?
    // it causes the position to go back to old_pos after a probe at probepos ?????
    //do_blocking_move_to(old_pos); 

    if (raise_after == PROBE_PT_STOW)
      probe.move_z_after_probing(); //TG move z-axis to Z_AFTER_PROBING position

    report_current_position();      // report position info
  }
  else { //TG tried to probe past bed limits
    SERIAL_ECHOLNF(GET_EN_TEXT_F(MSG_ZPROBE_OUT));
    LCD_MESSAGE(MSG_ZPROBE_OUT);  //TG send a host::notify message
  }

  probe.use_probing_tool(false);  //TG reset this state
}

#endif // HAS_BED_PROBE
