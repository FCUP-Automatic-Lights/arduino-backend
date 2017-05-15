#include "Automatic_Lights_Types.h"

void _envAnalysis(int, int);
void _reset_sensor(SensorInfo*, int);
void _reset_room(RoomInfo*);
void _deactivate_ir_sensors();

void _debug_room();
void _debug_ldr_sensor(RoomInfo*);
void _debug_ir_sensors(int, SensorInfo*);
