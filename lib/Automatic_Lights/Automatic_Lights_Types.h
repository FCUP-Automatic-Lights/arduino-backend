/*
 * Sensor Information
 */
struct SensorInfo {
    unsigned long timestamp;
    int status;
    int mode;
    int pin;
};

/*
 * Room Information
 */
struct RoomInfo {
    int person_count;
    int lux;
};