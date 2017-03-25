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
#define SHARP_UPPER_THRESHOLD 37

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
 * Wi-Fi WPA2 PSK Credentials
 */
const char* ssid = "asnf";
const char* pass = "asdjf12je3hepkrmddr56tg56w-klkijmrasnf";

/*
 * System State
 */
int wifi_status;
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

    digitalWrite(AL_LED_OUTPUT, LOW); // Still need to check what's wrong with this

    al_fsensor = malloc(sizeof(SensorInfo));
    al_ssensor = malloc(sizeof(SensorInfo));
    al_room = malloc(sizeof(RoomInfo));

    wifi_status = WL_IDLE_STATUS;
    al_ir_status = AL_STATUS_DEFAULT;

    while ( WiFi.status() == WL_NO_SHIELD ) { Serial.println("WiFi shield not present, please insert it to proceed the execution"); }

    while ( wifi_status != WL_CONNECTED ) {
        Serial.print("Connecting to "); Serial.println(ssid);

        wifi_status = WiFi.begin(ssid, pass);
    }

    // Setup Sensors
    al_fsensor->last_distance = SHARP_UPPER_THRESHOLD;
    al_fsensor->distance = 0;
    al_fsensor->pin = A1;

    al_ssensor->last_distance = SHARP_UPPER_THRESHOLD;
    al_ssensor->distance = 0;
    al_ssensor->pin = A2;
}

/*
 * System Procedures
 */
void loop () {
    if(al_fsensor->distance != 0 || al_ssensor->distance != 0) {
        al_ir_status = AL_STATUS_DEFAULT;
        al_fsensor->last_distance = al_fsensor->distance;
        al_ssensor->last_distance = al_ssensor->distance;
    }

    // Set the distance
    al_fsensor->distance = getDistance(al_fsensor->pin);
    al_ssensor->distance = getDistance(al_ssensor->pin);

    // Read lux value
    al_room->lux = al_ldr.read();

    debug_ldr_sensor(al_room->lux);
    debug_ir_sensors(1, al_fsensor->distance, al_fsensor->last_distance);
    debug_ir_sensors(2, al_ssensor->distance, al_ssensor->last_distance);

    if(al_room->lux < AL_MINIMUM_LUX) {
        digitalWrite(AL_LED_OUTPUT, HIGH);
    } else {
        digitalWrite(AL_LED_OUTPUT, LOW);
    }

    delay(AL_DELAY_MS);
}

uint16_t getDistance (uint16_t sensor) {
    return 13 * pow(analogRead(sensor) * 0.0048828125 , -1);
}

/*
 * System Debug
 */
void debug_ldr_sensor(uint16_t lux) {
    Serial.println();
    Serial.print("Lux Level: ");
    Serial.println(lux);
}

void debug_ir_sensors(uint16_t sensor, uint16_t distance, uint16_t last_distance) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" ");
    Serial.print(distance);
    Serial.print("cm");
    Serial.print(" ");
    Serial.print(last_distance);
    Serial.println("cm");
}