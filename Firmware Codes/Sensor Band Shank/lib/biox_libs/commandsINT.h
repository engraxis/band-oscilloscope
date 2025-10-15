#ifndef COMMAND_H
#define COMMAND_H

// Define the orders that can be sent and received
enum Command {
    INIT = 0,
    ACK = 1,
    START_TRANSMISSION = 2,
    STOP_TRANSMISSION = 3,
    GAIN_RESET = 4,
    CALIBRATE = 5,
    DISCONNECT = 6,
    SHUTDOWN = 7,
    ERROR = 8,
    NONE = 9,
    HEART_BEAT = 10,
    GET_STEP_COUNT = 11,
};

// Prepended to the data sent via bluetooth to allow the receiver to differentiate between different types of messages
enum ResponseCode {
    SENSOR_DATA = 2,
    STEP_COUNT = 11,
};
// Response code takes up 1 byte
const size_t RESPONSE_CODE_SIZE = 1;

typedef enum Command Command;

typedef enum ResponseCode ResponseCode;

#endif


//Python ->
// C - INIT ent only for the first loop
// x  - Shutdown
// S  - START Data request
// T  - STOP data req
// I  - Amplifier gain ++
// D - Gain --
// R - Gain reset
// C - calibrate

