/*
 * Sensor Information
 */
struct SensorInfo {
  long timestamp;
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
  int lights_on;
  int total_time;
  int lights_initial_time;
  int success;
};
