#include "Arduino.h"
#include <WiFi.h>
#include <TinkerKit.h>
#include <SPI.h>

/*
 * Entries - Exits
 */
#define AL_STATUS_IDLE -1
#define AL_STATUS_ENTRY 0
#define AL_STATUS_EXIT 1

/*
 * LUX - Ambient Light Sensor
 */
#define AL_MINIMUM_LUX 200

/*
 * LED's GPIO
 */
#define AL_LED_OUTPUT_DEBUG A5
#define AL_LED_OUTPUT_RED 10
#define AL_LED_OUTPUT_GREEN 9
#define AL_LED_OUTPUT_BLUE 8

/*
 * WPA2 PSK
 */
#ifdef COMPILE_WITH_SSID

#define AL_SSID "some_ssid"
#define AL_PASS "some_pass"

#endif

/*
 * VENDOR / DISTANCE
 */
#define SHARP_AL_DISTANCE 11
#define SHARP_UPPER_THRESHOLD 37
#define SHARP_LOWER_THRESHoLD 4

/*
 * MISC
 */
#define AL_DELAY_MS 750

/*
 * Sensor Information
 */
struct SensorInfo {
    unsigned long timestamp;
    uint16_t status;
    uint16_t pin;
};

/*
 * Room Information
 */
struct RoomInfo {
    uint16_t person_count;
    uint16_t lux;
};

/*
 * System State
 */

#ifdef COMPILE_WITH_SSID
int wifi_status;
#endif

SensorInfo* al_fsensor;
SensorInfo* al_ssensor;
RoomInfo* al_room;
TKLightSensor al_ldr(I0);

void _blink(uint16_t times);
void _reset_sensor(SensorInfo* s, uint16_t pin);
void _set_color(uint16_t red, uint16_t green, uint16_t blue);

void _debug_ldr_sensor(RoomInfo* room);
void _debug_ir_sensors(uint16_t pin, SensorInfo* sensor);

/*
 * Boot System
 */
void setup() {
    Serial.begin(9600);

    pinMode(AL_LED_OUTPUT_DEBUG, OUTPUT);
    pinMode(AL_LED_OUTPUT_RED, OUTPUT);
    pinMode(AL_LED_OUTPUT_GREEN, OUTPUT);
    pinMode(AL_LED_OUTPUT_BLUE, OUTPUT);

    // Allocate space on the heap for sensor information
    al_fsensor = (SensorInfo*) malloc(sizeof(SensorInfo));
    al_ssensor = (SensorInfo*) malloc(sizeof(SensorInfo));
    al_room = (RoomInfo*) malloc(sizeof(RoomInfo));

#ifdef COMPILE_WITH_SSID
    // Keep state of the current status of wifi board.
    wifi_status = WL_IDLE_STATUS;

    while ( WiFi.status() == WL_NO_SHIELD ) { Serial.println("WiFi shield not present, please insert it to proceed the execution"); }

    while ( wifi_status != WL_CONNECTED ) {
        Serial.print("Connecting to "); Serial.println(AL_SSID);

        wifi_status = WiFi.begin(AL_SSID, AL_PASS);
    }
#endif

    // Setup Sensors
    _reset_sensor(al_fsensor, A1);
    _reset_sensor(al_ssensor, A2);

    _blink(3);
}

/*
 * System Procedures
 */
void loop () {
    // Set the detection (WIP / Placeholder: Need new sensors to develop this)
    al_fsensor->timestamp = millis();
    al_ssensor->timestamp = millis();

    // Read Lux value
    al_room->lux = al_ldr.read();

    // Debug
    _debug_ldr_sensor(al_room);
    _debug_ir_sensors(1, al_fsensor);
    _debug_ir_sensors(2, al_ssensor);

    // Light off - Light on
    if(al_room->lux < AL_MINIMUM_LUX) {
        _set_color(135, 206, 235); // SkyBlue
    } else {
        _set_color(0, 0, 0);
    }

    // Slow down a bit the execution time,
    delay(AL_DELAY_MS);
}

void _blink(uint16_t times) {
    while(times != 0) {
        digitalWrite(AL_LED_OUTPUT_DEBUG, HIGH);
        delay(200);
        digitalWrite(AL_LED_OUTPUT_DEBUG, LOW);
        delay(200);
        times--;
    }
}

void _reset_sensor(SensorInfo* s, uint16_t pin) {
    s->status = AL_STATUS_IDLE;
    s->timestamp = 0;
    s->pin = pin;
}

/*
 * System Debug
 */
void _debug_ldr_sensor(RoomInfo* room) {
    Serial.println("------------------------------");
    Serial.print("Lux Level: ");
    Serial.println(room->lux);
}

void _debug_ir_sensors(uint16_t pin, SensorInfo* sensor) {
    char _to_print[100];

    sprintf(_to_print, "Sensor %d -> Last Dectection: %lu ms", pin, sensor->timestamp, sensor->status);

    Serial.println(_to_print);
}

void _set_color(uint16_t red, uint16_t green, uint16_t blue) {
  analogWrite(AL_LED_OUTPUT_RED, red);
  analogWrite(AL_LED_OUTPUT_GREEN, green);
  analogWrite(AL_LED_OUTPUT_BLUE, blue); 
}