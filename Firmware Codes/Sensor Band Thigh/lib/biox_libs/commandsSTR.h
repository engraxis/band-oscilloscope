#ifndef COMMAND_H
#define COMMAND_H

// Define the orders that can be sent and received
enum Command {
    INIT = '0',
    ACK = '1',
    START_TRANSMISSION = '2',
    STOP_TRANSMISSION = '3',
    GAIN_RESET = '4',
    CALIBRATE = '5',
    DISCONNECT = '6',
    ERROR = '8',
    NONE = '9',
};

typedef enum Command Command;

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

