#include "Automatic_Lights_Types.h"

void _blink(int);
void _reset_sensor(SensorInfo*, int);
void _reset_room(RoomInfo*);
void _set_color(int, int, int);
void _deactivate_ir_sensors();

void _debug_connection();
void _debug_room();
void _debug_ldr_sensor(RoomInfo*);
void _debug_ir_sensors(int, SensorInfo*);