#include <WiFi.h>
#include <TinkerKit.h>

/*
 * Arduino Parameters
 */
#define AL_STATUS_DEFAULT -1
#define AL_STATUS_ENTRY 0
#define AL_STATUS_EXIT 1
#define AL_DISTANCE 11
#define AL_MINIMUM_LUX 200
#define AL_DELAY_MS 750
#define AL_LED_OUTPUT 10

//#define AL_SSID "some_ssid"
//#define AL_PASS "some_pass"

/*
 * Vendor Parameters
 */
#define SHARP_UPPER_THRESHOLD 37
#define SHARP_LOWER_THRESHoLD 4

/*
 * Fsensor => PORT A1 => Sensor inside room
 * Ssensor => PORT A2 => Sensor outside room
 *
 * LED => PORT 10
 * LDR Sensor => PORT I0
 */

/*
 * Sensor Information
 */
typedef struct SensorInfo {
    uint16_t last_timestamp;
    uint16_t last_distance;
    uint16_t timestamp;
    uint16_t distance;
    uint16_t pin;
};

/*
 * Room Information
 */
typedef struct RoomInfo {
    uint16_t person_count;
    uint16_t lux;
};

/*
 * System State
 */

#ifdef AL_SSID
int wifi_status;
#endif

int al_ir_status;
SensorInfo* al_fsensor;
SensorInfo* al_ssensor;
RoomInfo* al_room;
TKLightSensor al_ldr(I0);

/*
 * Boot System
 */
void setup() {
    Serial.begin(9600);
    pinMode(AL_LED_OUTPUT, OUTPUT);

    // Allocate space on the heap for sensor information
    al_fsensor = malloc(sizeof(SensorInfo));
    al_ssensor = malloc(sizeof(SensorInfo));
    al_room = malloc(sizeof(RoomInfo));

    // Keep state of the current status of system.
    al_ir_status = AL_STATUS_DEFAULT;

#ifdef AL_SSID
    // Keep state of the current status of system.
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

    _blink(10);
}

void _blink(uint16_t times) {
    while(times != 0) {
        digitalWrite(AL_LED_OUTPUT, HIGH);
        delay(200);
        digitalWrite(AL_LED_OUTPUT, LOW);
        delay(200);
        times--;
    }
}

/*
 * System Procedures
 */
void loop () {
    // We should keep some history
    if(al_fsensor->distance != 0 || al_ssensor->distance != 0) {
        al_ir_status = AL_STATUS_DEFAULT;

        al_fsensor->last_distance = al_fsensor->distance;
        al_fsensor->last_timestamp = al_fsensor->timestamp;
        al_ssensor->last_distance = al_ssensor->distance;
        al_ssensor->last_timestamp = al_ssensor->timestamp;
    }

    // Set the distance
    al_fsensor->distance = _getDistance(al_fsensor->pin);
    al_fsensor->timestamp = millis();
    al_ssensor->distance = _getDistance(al_ssensor->pin);
    al_ssensor->timestamp = millis();

    // Read lux value
    al_room->lux = al_ldr.read();

    // Debug
    _debug_ldr_sensor(al_room);
    _debug_ir_sensors(1, al_fsensor);
    _debug_ir_sensors(2, al_ssensor);

    // Light off - Light on
    if(al_room->lux < AL_MINIMUM_LUX) {
        digitalWrite(AL_LED_OUTPUT, HIGH);
    } else {
        digitalWrite(AL_LED_OUTPUT, LOW);
    }

    // Slow down a bit the execution time,
    delay(AL_DELAY_MS);
}

uint16_t _getDistance(uint16_t sensor) {
    return 13 * pow(analogRead(sensor) * 0.0048828125 , -1);
}

void _reset_sensor(SensorInfo* s, uint16_t pin) {
    s->last_distance = SHARP_UPPER_THRESHOLD;
    s->last_timestamp = 0;
    s->distance = 0;
    s->timestamp = 0;
    s->pin = pin;
}

/*
 * System Debug
 */
void _debug_ldr_sensor(RoomInfo* room) {
    Serial.println();
    Serial.print("Lux Level: ");
    Serial.println(room->lux);
}

void _debug_ir_sensors(uint16_t pin, SensorInfo* sensor) {
    Serial.print("Sensor ");
    Serial.print(pin);
    Serial.print(" ");
    Serial.print(sensor->distance);
    Serial.print("cm ");
    Serial.print(sensor->timestamp);
    Serial.print(" ms ");
    Serial.print(sensor->last_distance);
    Serial.print("cm ");
    Serial.print(sensor->last_timestamp);
    Serial.println(" ms");

}