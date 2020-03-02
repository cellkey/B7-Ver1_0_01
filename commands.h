/*
 * commands.h
 *
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_
#include <stdint.h>

// I/O alert commands (from device to server)
#define 0x01 ALERT_1 // kando buttle state: 0 - Empty, 1 - Full
#define 0x02
#define 0x03

// Activities (from server to device)
#define 0x20 RESET_BOTTLE_STATE // 0 - set bottle state as empty, 1 - full 


// Online state command
#define 0x40 RELAY1_ACTIVE // PUMP is connected to relay1: 0 - turn pump off, 1 - turn pump on
#define 0x41 RELAY2_ACTIVE // values: <duration - seconds>

//Sensor TH-levels (from device to server - on quary request, or from server to device update values)
#define 0x60 TEMP_TH // values: <TH MAX>
#define 0x61 PH_TH   // values: <PH MIN><PH_MAX><PH_BOTTLE_MIN><PH_BOTTLE_MAX>
#define 0x62 EC_TH   // values: <EC_MIN><EC_MAX><EC_BOTTLE_MIN><EC_BOTTLE_MAX>
#define 0x63 WATER_LEVEL // values: <WATER_LEVEL_HIGHT>, unit:cm



#endif /* COMMANDS_H_ */ 