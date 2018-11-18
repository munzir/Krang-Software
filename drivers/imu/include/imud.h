/**
 * A simple interface to Linux imu devies
 *
 * @file js.h
 * @author  martin
 */


#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "ssdmu/ssdmu.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CHANNEL_NAME "imu-data"
#define IMU_CHANNEL_SIZE 54

#ifdef __cplusplus
}
#endif

#endif // JS_H
