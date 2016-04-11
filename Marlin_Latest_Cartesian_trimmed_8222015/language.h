#ifndef LANGUAGE_H
#define LANGUAGE_H

// NOTE: IF YOU CHANGE THIS FILE / MERGE THIS FILE WITH CHANGES
//
//   ==> ALWAYS TRY TO COMPILE MARLIN WITH/WITHOUT "ULTIPANEL" / "ULTRALCD" / "SDSUPPORT" #define IN "Configuration.h" 
//   ==> ALSO TRY ALL AVAILABLE "LANGUAGE_CHOICE" OPTIONS

// Languages
// 1  English

#ifndef LANGUAGE_CHOICE
	#define LANGUAGE_CHOICE 1  // Pick your language from the list above
#endif

#define PROTOCOL_VERSION "1.0"

	#ifdef CUSTOM_MENDEL_NAME
		#define MACHINE_NAME CUSTOM_MENDEL_NAME
	#else
		#define MACHINE_NAME "Mendel"
	#endif

// Default firmware set to Mendel
	#define FIRMWARE_URL "https://github.com/ErikZalm/Marlin/"


#ifndef MACHINE_UUID
   #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#endif


#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)


// Common LCD messages
/* nothing here as of yet */

// Common serial messages
#define MSG_MARLIN "Marlin"



#if LANGUAGE_CHOICE == 1

// LCD Menu Messages
// Please note these are limited to 17 characters!

	#define WELCOME_MSG MACHINE_NAME " ready."
	#define MSG_SD_INSERTED "Card inserted"
	#define MSG_SD_REMOVED "Card removed"
	#define MSG_MAIN "Main"
	#define MSG_AUTOSTART "Autostart"
	#define MSG_DISABLE_STEPPERS "Disable steppers"
	#define MSG_AUTO_HOME "Auto home"
	#define MSG_SET_ORIGIN "Set origin"
	#define MSG_PREHEAT_PLA "Preheat PLA"
	#define MSG_PREHEAT_PLA0 "Preheat PLA 1"
	#define MSG_PREHEAT_PLA1 "Preheat PLA 2"
	#define MSG_PREHEAT_PLA2 "Preheat PLA 3"
	#define MSG_PREHEAT_PLA012 "Preheat PLA All"
	#define MSG_PREHEAT_PLA_BEDONLY "Preheat PLA Bed"
	#define MSG_PREHEAT_PLA_SETTINGS "Preheat PLA conf"
	#define MSG_PREHEAT_ABS "Preheat ABS"
	#define MSG_PREHEAT_ABS0 "Preheat ABS 1"
	#define MSG_PREHEAT_ABS1 "Preheat ABS 2"
	#define MSG_PREHEAT_ABS2 "Preheat ABS 3"
	#define MSG_PREHEAT_ABS012 "Preheat ABS All"
	#define MSG_PREHEAT_ABS_BEDONLY "Preheat ABS Bed"
	#define MSG_PREHEAT_ABS_SETTINGS "Preheat ABS conf"
	#define MSG_COOLDOWN "Cooldown"
	#define MSG_SWITCH_PS_ON "Switch power on"
	#define MSG_SWITCH_PS_OFF "Switch power off"
	#define MSG_EXTRUDE "Extrude"
	#define MSG_RETRACT "Retract"
	#define MSG_MOVE_AXIS "Move axis"
	#define MSG_MOVE_X "Move X"
	#define MSG_MOVE_Y "Move Y"
	#define MSG_MOVE_Z "Move Z"
	#define MSG_MOVE_E "Extruder"
	#define MSG_MOVE_E1 "Extruder2"
	#define MSG_MOVE_E2 "Extruder3"
	#define MSG_MOVE_01MM "Move 0.1mm"
	#define MSG_MOVE_1MM "Move 1mm"
	#define MSG_MOVE_10MM "Move 10mm"
	#define MSG_SPEED "Speed"
	#define MSG_NOZZLE "Nozzle"
	#define MSG_NOZZLE1 "Nozzle2"
	#define MSG_NOZZLE2 "Nozzle3"
	#define MSG_BED "Bed"
	#define MSG_FAN_SPEED "Fan speed"
	#define MSG_FLOW "Flow"
	#define MSG_FLOW0 "Flow 0"
	#define MSG_FLOW1 "Flow 1"
	#define MSG_FLOW2 "Flow 2"
	#define MSG_CONTROL "Control"
	#define MSG_MIN " \002 Min"
	#define MSG_MAX " \002 Max"
	#define MSG_FACTOR " \002 Fact"
	#define MSG_AUTOTEMP "Autotemp"
	#define MSG_ON "On "
	#define MSG_OFF "Off"
	#define MSG_PID_P "PID-P"
	#define MSG_PID_I "PID-I"
	#define MSG_PID_D "PID-D"
	#define MSG_PID_C "PID-C"
	#define MSG_ACC  "Accel"
	#define MSG_VXY_JERK "Vxy-jerk"
	#define MSG_VZ_JERK "Vz-jerk"
	#define MSG_VE_JERK "Ve-jerk"
	#define MSG_VMAX "Vmax "
	#define MSG_X "x"
	#define MSG_Y "y"
	#define MSG_Z "z"
	#define MSG_E "e"
	#define MSG_VMIN "Vmin"
	#define MSG_VTRAV_MIN "VTrav min"
	#define MSG_AMAX "Amax "
	#define MSG_A_RETRACT "A-retract"
	#define MSG_XSTEPS "Xsteps/mm"
	#define MSG_YSTEPS "Ysteps/mm"
	#define MSG_ZSTEPS "Zsteps/mm"
	#define MSG_ESTEPS "Esteps/mm"
	#define MSG_RECTRACT "Rectract"
	#define MSG_TEMPERATURE "Temperature"
	#define MSG_MOTION "Motion"
	#define MSG_CONTRAST "LCD contrast"
	#define MSG_STORE_EPROM "Store memory"
	#define MSG_LOAD_EPROM "Load memory"
	#define MSG_RESTORE_FAILSAFE "Restore failsafe"
	#define MSG_REFRESH "Refresh"
	#define MSG_WATCH "Info screen"
	#define MSG_PREPARE "Prepare"
	#define MSG_TUNE "Tune"
	#define MSG_PAUSE_PRINT "Pause print"
	#define MSG_RESUME_PRINT "Resume print"
	#define MSG_STOP_PRINT "Stop print"
	#define MSG_CARD_MENU "Print from SD"
	#define MSG_NO_CARD "No SD card"
	#define MSG_DWELL "Sleep..."
	#define MSG_USERWAIT "Wait for user..."
	#define MSG_RESUMING "Resuming print"
	#define MSG_NO_MOVE "No move."
	#define MSG_KILLED "KILLED. "
	#define MSG_STOPPED "STOPPED. "
	#define MSG_CONTROL_RETRACT  "Retract mm"
	#define MSG_CONTROL_RETRACT_SWAP  "Swap Re.mm"
	#define MSG_CONTROL_RETRACTF "Retract  V"
	#define MSG_CONTROL_RETRACT_ZLIFT "Hop mm"
	#define MSG_CONTROL_RETRACT_RECOVER "UnRet +mm"
	#define MSG_CONTROL_RETRACT_RECOVER_SWAP "S UnRet+mm"
	#define MSG_CONTROL_RETRACT_RECOVERF "UnRet  V"
	#define MSG_AUTORETRACT "AutoRetr."
	#define MSG_FILAMENTCHANGE "Change filament"
	#define MSG_INIT_SDCARD "Init. SD card"
	#define MSG_CNG_SDCARD "Change SD card"
	#define MSG_ZPROBE_OUT "Z probe out. bed"
	#define MSG_POSITION_UNKNOWN "Home X/Y before Z"
	#define MSG_ZPROBE_ZOFFSET "Z Offset"
	#define MSG_BABYSTEP_X "Babystep X"
	#define MSG_BABYSTEP_Y "Babystep Y"
	#define MSG_BABYSTEP_Z "Babystep Z"
	#define MSG_ENDSTOP_ABORT "Endstop abort"

// Serial Console Messages

	#define MSG_Enqueing "enqueing \""
	#define MSG_POWERUP "PowerUp"
	#define MSG_EXTERNAL_RESET " External Reset"
	#define MSG_BROWNOUT_RESET " Brown out Reset"
	#define MSG_WATCHDOG_RESET " Watchdog Reset"
	#define MSG_SOFTWARE_RESET " Software Reset"
	#define MSG_AUTHOR " | Author: "
	#define MSG_CONFIGURATION_VER " Last Updated: "
	#define MSG_FREE_MEMORY " Free Memory: "
	#define MSG_PLANNER_BUFFER_BYTES "  PlannerBufferBytes: "
	#define MSG_OK "ok"
	#define MSG_FILE_SAVED "Done saving file."
	#define MSG_ERR_LINE_NO "Line Number is not Last Line Number+1, Last Line: "
	#define MSG_ERR_CHECKSUM_MISMATCH "checksum mismatch, Last Line: "
	#define MSG_ERR_NO_CHECKSUM "No Checksum with line number, Last Line: "
	#define MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM "No Line Number with checksum, Last Line: "
	#define MSG_FILE_PRINTED "Done printing file"
	#define MSG_BEGIN_FILE_LIST "Begin file list"
	#define MSG_END_FILE_LIST "End file list"
	#define MSG_M104_INVALID_EXTRUDER "M104 Invalid extruder "
	#define MSG_M105_INVALID_EXTRUDER "M105 Invalid extruder "
	#define MSG_M200_INVALID_EXTRUDER "M200 Invalid extruder "
	#define MSG_M218_INVALID_EXTRUDER "M218 Invalid extruder "
	#define MSG_M221_INVALID_EXTRUDER "M221 Invalid extruder "
	#define MSG_ERR_NO_THERMISTORS "No thermistors - no temperature"
	#define MSG_M109_INVALID_EXTRUDER "M109 Invalid extruder "
	#define MSG_HEATING "Heating..."
	#define MSG_HEATING_COMPLETE "Heating done."
	#define MSG_BED_HEATING "Bed Heating."
	#define MSG_BED_DONE "Bed done."
	#define MSG_M115_REPORT "FIRMWARE_NAME:Marlin V1; Sprinter/grbl mashup for gen6 FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:" MACHINE_NAME " EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
	#define MSG_COUNT_X " Count X: "
	#define MSG_ERR_KILLED "Printer halted. kill() called!"
	#define MSG_ERR_STOPPED "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
	#define MSG_RESEND "Resend: "
	#define MSG_UNKNOWN_COMMAND "Unknown command: \""
	#define MSG_ACTIVE_EXTRUDER "Active Extruder: "
	#define MSG_INVALID_EXTRUDER "Invalid extruder"
	#define MSG_X_MIN "x_min: "
	#define MSG_X_MAX "x_max: "
	#define MSG_Y_MIN "y_min: "
	#define MSG_Y_MAX "y_max: "
	#define MSG_Z_MIN "z_min: "
	#define MSG_Z_MAX "z_max: "
	#define MSG_M119_REPORT "Reporting endstop status"
	#define MSG_ENDSTOP_HIT "TRIGGERED"
	#define MSG_ENDSTOP_OPEN "open"
	#define MSG_HOTEND_OFFSET "Hotend offsets:"

	#define MSG_SD_CANT_OPEN_SUBDIR "Cannot open subdir"
	#define MSG_SD_INIT_FAIL "SD init fail"
	#define MSG_SD_VOL_INIT_FAIL "volume.init failed"
	#define MSG_SD_OPENROOT_FAIL "openRoot failed"
	#define MSG_SD_CARD_OK "SD card ok"
	#define MSG_SD_WORKDIR_FAIL "workDir open failed"
	#define MSG_SD_OPEN_FILE_FAIL "open failed, File: "
	#define MSG_SD_FILE_OPENED "File opened: "
	#define MSG_SD_SIZE " Size: "
	#define MSG_SD_FILE_SELECTED "File selected"
	#define MSG_SD_WRITE_TO_FILE "Writing to file: "
	#define MSG_SD_PRINTING_BYTE "SD printing byte "
	#define MSG_SD_NOT_PRINTING "Not SD printing"
	#define MSG_SD_ERR_WRITE_TO_FILE "error writing to file"
	#define MSG_SD_CANT_ENTER_SUBDIR "Cannot enter subdir: "

	#define MSG_STEPPER_TOO_HIGH "Steprate too high: "
	#define MSG_ENDSTOPS_HIT "endstops hit: "
	#define MSG_ERR_COLD_EXTRUDE_STOP " cold extrusion prevented"
	#define MSG_ERR_LONG_EXTRUDE_STOP " too long extrusion prevented"
	#define MSG_BABYSTEPPING_X "Babystepping X"
	#define MSG_BABYSTEPPING_Y "Babystepping Y"
	#define MSG_BABYSTEPPING_Z "Babystepping Z"
	#define MSG_SERIAL_ERROR_MENU_STRUCTURE "Error in menu structure"

#endif
#endif // ifndef LANGUAGE_H
