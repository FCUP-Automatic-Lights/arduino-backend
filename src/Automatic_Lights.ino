#include <TinkerKit.h>
#include "../lib/Automatic_Lights/Automatic_Lights.h"
#include "Arduino.h"

/*
 * Entries - Exits
 */
#define AL_ACTIVATED 1
#define AL_DEACTIVATED 0
#define AL_SENSOR_RESET_THRESHOLD_MS 3000

/*
 * IR Sensors
 */
#define AL_FSENSOR_THRESHOLD 750
#define AL_SSENSOR_THRESHOLD 750

/*
 * LUX - Ambient Light Sensor
 */
#define AL_LDR I0
#define AL_MINIMUM_LUX 450

/*
 * LIGHT GPIO
 */
#define AL_LIGHT 2

/*
 * MISC
 */
#define AL_DELAY_MS 500

SensorInfo *al_fsensor;
SensorInfo *al_ssensor;
RoomInfo *al_room;
TKLightSensor al_ldr(AL_LDR);

int registered_user;
int turn_on;
String incomingByte;

/*
 * Boot System
 */
void setup() {
    Serial.begin(9600);

    // LIGHT should run on OUTPUT MODE
    pinMode(AL_LIGHT, OUTPUT);

    // Allocate space on the heap for sensor information
    al_fsensor = (SensorInfo *)malloc(sizeof(SensorInfo));
    al_ssensor = (SensorInfo *)malloc(sizeof(SensorInfo));
    al_room    = (RoomInfo *)malloc(sizeof(RoomInfo));

    // Setup Sensors
    _reset_sensor(al_fsensor, A2);
    _reset_sensor(al_ssensor, A3);
    _reset_room(al_room);

    // Initial Parameters
    registered_user = 1;
    turn_on = 0;
}

/*
 * System Procedures
 */
void loop() {
    if (Serial.available() > 0) {
      incomingByte = Serial.readString();

      registered_user = incomingByte.charAt(0) - '0';
      turn_on = incomingByte.charAt(2) - '0';
    }

    _envAnalysis(registered_user, turn_on);

    _debug_room();
    _debug_ir_sensors(1, al_fsensor);
    _debug_ir_sensors(2, al_ssensor);

    delay(AL_DELAY_MS);
}

void _reset_room(RoomInfo *r) {
    r->person_count        = 0;
    r->lux                 = 0;
    r->total_time          = 0;
    r->lights_initial_time = 0;
    r->remote              = 0;
    r->lights_on           = AL_ACTIVATED;
}

void _envAnalysis(int registered_user, int turn_on) {
    int current_time = millis();

    uint16_t fsensor_reset_threshold = al_fsensor->timestamp + AL_SENSOR_RESET_THRESHOLD_MS;
    uint16_t ssensor_reset_threshold = al_ssensor->timestamp + AL_SENSOR_RESET_THRESHOLD_MS;

    if ((fsensor_reset_threshold <= current_time && al_fsensor->mode == AL_ACTIVATED) ||
        (ssensor_reset_threshold <= current_time && al_ssensor->mode == AL_ACTIVATED)) {

        _deactivate_ir_sensors();
    }

    al_fsensor->status = analogRead(al_fsensor->pin);
    al_ssensor->status = analogRead(al_ssensor->pin);

    if (al_fsensor->status >= AL_FSENSOR_THRESHOLD && al_fsensor->mode == AL_DEACTIVATED) {
        al_fsensor->timestamp = current_time;
        al_fsensor->mode      = AL_ACTIVATED;
    }

    if (al_ssensor->status >= AL_SSENSOR_THRESHOLD && al_ssensor->mode == AL_DEACTIVATED) {
        al_ssensor->timestamp = current_time;
        al_ssensor->mode      = AL_ACTIVATED;
    }

    if (al_fsensor->mode == AL_ACTIVATED && al_ssensor->mode == AL_ACTIVATED) {
        if (al_fsensor->timestamp < al_ssensor->timestamp) {
            al_room->person_count++;
        } else if (al_room->person_count > 0) {
            al_room->person_count--;
        }

        _deactivate_ir_sensors();
    }

    al_room->lux = al_ldr.read();

    if(al_room->lux < AL_MINIMUM_LUX && ((al_room->person_count >= 1 && registered_user == 1 && turn_on == 0) || (turn_on == 1 && registered_user == 0)) && al_room->lights_on == AL_DEACTIVATED) {
        digitalWrite(AL_LIGHT, HIGH);
        al_room->lights_on = AL_ACTIVATED;

        if (turn_on == 1) {
            al_room->remote = 1;
        }

        al_room->lights_initial_time = millis();
    } else if (al_room->lights_on == AL_ACTIVATED && (al_room->lux >= AL_MINIMUM_LUX || (al_room->person_count == 0 && al_room->remote != 1) || (registered_user == 0 && al_room->remote != 1) || (registered_user == 0 && turn_on == 2))) {
        digitalWrite(AL_LIGHT, LOW);
        al_room->lights_on = AL_DEACTIVATED;
        al_room->remote = 0;
        al_room->total_time = millis() - al_room->lights_initial_time;
    }
}

void _reset_sensor(SensorInfo *s, int pin) {
    s->status    = 0;
    s->timestamp = 0;
    s->mode      = AL_DEACTIVATED;
    s->pin       = pin;
}

void _deactivate_ir_sensors() {
    al_fsensor->timestamp = 0;
    al_ssensor->timestamp = 0;
    al_fsensor->mode      = AL_DEACTIVATED;
    al_ssensor->mode      = AL_DEACTIVATED;
}

/*
 * System Debug
 */
void _debug_ir_sensors(int pin, SensorInfo *sensor) {
    char _to_print[100];

    sprintf(_to_print, "S%d : %lu : %d", pin, sensor->timestamp, sensor->status);

    Serial.println(_to_print);
}

void _debug_room() {
    char _to_print[100];
    int total_time = al_room->lights_on == AL_DEACTIVATED && al_room->total_time > 10 ? al_room->total_time : 0;

    sprintf(_to_print, "PC : %d | L : %d | T : %d | S : %d", al_room->person_count, al_room->lux, total_time, al_room->lights_on);

    if(total_time) {
        al_room->total_time = 0;
    }

    Serial.println(_to_print);
}
