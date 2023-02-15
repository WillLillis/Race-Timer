#if !defined(_TIMER_MSG_H)
#define _TIMER_MSG_H
#include <stdint.h>

typedef enum timer_status_t : int32_t {
    TIMER_MSG_ERR = -5,               // some kind of (esp_now) messaging error occured
    TIMER_SL_LASER_NALIGN = -4,       // starting line laser is not aligned properly
    TIMER_FL_LASER_NALIGN = -3,       // finish line laser is not aligned properly
    TIMER_NREADY = -2,                // start line module indicates it's NOT ready for a run
    TIMER_ERROR = -1,                 // generic ERROR status
    TIMER_OK = 0,                     // generic 'OK' status
    TIMER_RUN_ENQ,                    // finish line module asks the starting line if it's ready for a run
    TIMER_READY,                      // start line module indicates it's ready for a run
    TIMER_DATA,                       // timer module indicates it's sending a timing data payload
    TIMER_SYNC_PULSE_RCV,             // start line module indicates it received the sync pulse
    TIMER_TEST                        // test status, no meaning
}timer_status_t;

typedef struct timer_msg{
    timer_status_t status;
    uint64_t time;
}timer_msg;

#endif // _TIMER_MSG_H include guard