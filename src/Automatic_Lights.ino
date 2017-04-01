#include "Arduino.h"
#include <WiFi.h>
#include <TinkerKit.h>
#include <SPI.h>
#include "../lib/Automatic_Lights/Automatic_Lights.h"

/*
 * Entries - Exits
 */
#define AL_ACTIVATED 1
#define AL_DEACTIVATED 0
#define AL_SENSOR_RESET_THRESHOLD_MS 3000

/*
 * IR Sensors
 */
#define AL_FSENSOR_THRESHOLD 1000
#define AL_SSENSOR_THRESHOLD 1000

/*
 * LUX - Ambient Light Sensor
 */
#define AL_LDR I0
#define AL_MINIMUM_LUX 450

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
#ifdef DEBUG

#define AL_SSID "Redmi"
#define AL_PASS "1234567890."
#define AL_ENDPOINT "arduino.cc"

#endif

/*
 * MISC
 */
#define AL_DELAY_MS 250

/*
 * System State
 */
#ifdef DEBUG
int wifi_status;
WiFiClient client;
#endif

SensorInfo* al_fsensor;
SensorInfo* al_ssensor;
RoomInfo* al_room;
TKLightSensor al_ldr(AL_LDR);

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

#ifdef DEBUG
    Keep state of the current status of wifi board.
    wifi_status = WL_IDLE_STATUS;

    while ( WiFi.status() == WL_NO_SHIELD ) { Serial.println("WiFi shield not present, please insert it to proceed the execution"); }

    while ( wifi_status != WL_CONNECTED ) {
        Serial.print("Connecting to "); Serial.println(AL_SSID);

        wifi_status = WiFi.begin(AL_SSID, AL_PASS);

        delay(2500);
    }

    Serial.println("\nStarting connection to server...");
    if(client.connect(AL_ENDPOINT, 80)) {
        Serial.println("connected to server");
        
        client.println("GET /latest.txt HTTP/1.1");
        client.println("Host: www.arduino.cc");
        client.println("User-Agent: ArduinoWiFi/1.1");
        client.println("Connection: close");
        client.println();

    }
#endif

    // Setup Sensors
    _reset_sensor(al_fsensor, A3);
    _reset_sensor(al_ssensor, A4);
    _reset_room(al_room);

    // Mark system as ready so user can start the interaction
    _blink(3);
}

/*
 * System Procedures
 */
void loop () {
    int current_time = millis();

    if ((al_fsensor->timestamp + AL_SENSOR_RESET_THRESHOLD_MS <= current_time && al_fsensor->mode == AL_ACTIVATED) || 
        (al_ssensor->timestamp + AL_SENSOR_RESET_THRESHOLD_MS <= current_time && al_ssensor->mode == AL_ACTIVATED)) {
            _deactivate_ir_sensors();
    }

    al_fsensor->status = analogRead(al_fsensor->pin);
    al_ssensor->status = analogRead(al_ssensor->pin);

    if (al_fsensor->status >= AL_FSENSOR_THRESHOLD && al_fsensor->mode == AL_DEACTIVATED) {
        al_fsensor->timestamp = current_time;
        al_fsensor->mode = AL_ACTIVATED;
    }

    if (al_ssensor->status >= AL_SSENSOR_THRESHOLD && al_ssensor->timestamp == AL_DEACTIVATED) {
        al_ssensor->timestamp = current_time;
        al_ssensor->mode = AL_ACTIVATED;
    }

    if (al_fsensor->mode == AL_ACTIVATED && al_ssensor->mode == AL_ACTIVATED) {
        if(al_fsensor->timestamp < al_ssensor->timestamp) {
            al_room->person_count++;
        } else if(al_room->person_count > 0) {
            al_room->person_count--;
        }

        _deactivate_ir_sensors();
    }

    al_room->lux = al_ldr.read();

#ifdef DEBUG
    _debug_room();
    _debug_ir_sensors(1, al_fsensor);
    _debug_ir_sensors(2, al_ssensor);

    _debug_connection();
#endif

    if(al_room->lux < AL_MINIMUM_LUX && al_room->person_count >= 1) {
        _set_color(0,0, 255);
    } else {
        _set_color(0, 0, 0);
    }

#ifdef DEBUG
    delay(AL_DELAY_MS);
#endif
}

void _blink(int times) {
    while(times != 0) {
        digitalWrite(AL_LED_OUTPUT_DEBUG, HIGH);
        delay(200);
        digitalWrite(AL_LED_OUTPUT_DEBUG, LOW);
        delay(200);
        times--;
    }
}

void _set_color(int red, int green, int blue) {
  analogWrite(AL_LED_OUTPUT_RED, red);
  analogWrite(AL_LED_OUTPUT_GREEN, green);
  analogWrite(AL_LED_OUTPUT_BLUE, blue); 
}

void _reset_room(RoomInfo* r) {
    r->person_count = 0;
    r->lux = 0;
}


void _reset_sensor(SensorInfo* s, int pin) {
    s->status = 0;
    s->timestamp = 0;
    s->mode = AL_DEACTIVATED;
    s->pin = pin;
}

void _deactivate_ir_sensors() {
    al_fsensor->timestamp = 0;
    al_ssensor->timestamp = 0;
    al_fsensor->mode = AL_DEACTIVATED;
    al_ssensor->mode = AL_DEACTIVATED;
}

/*
 * System Debug
 */
#ifdef DEBUG
void _debug_ldr_sensor(RoomInfo* room) {
    Serial.println("------------------------------");
    Serial.print("Lux Level: ");
    Serial.println(room->lux);
}

void _debug_ir_sensors(int pin, SensorInfo* sensor) {
    char _to_print[100];

    sprintf(_to_print, "Sensor %d -> Last Detection: %lu ms -> Status: %d", pin, sensor->timestamp, sensor->status);

    Serial.println(_to_print);
}

void _debug_room() {
    char _to_print[100];

    sprintf(_to_print, "Room -> Person Count: %d -> Lux: %d", al_room->person_count, al_room->lux);

    Serial.println(_to_print);
}

void _debug_connection() {
    while (client.available()) {
        Serial.println("Game Over");
        Serial.write(client.read());
    }

    if (!client.connected()) {
        Serial.println();
        Serial.print("Finished Polling from ");
        Serial.println(AL_ENDPOINT);
        client.stop();
    }
}
#endif