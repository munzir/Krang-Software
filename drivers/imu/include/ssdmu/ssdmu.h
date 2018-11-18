/**
 * Declarations for Silicon Sensing Dynamics Measurement Unit library
 *
 * @file ssdmu.h
 * @author martin, based on jon olson
 */

#ifndef _SSDMU_H_
#define _SSDMU_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include <ntcan.h>

/**
 * Packed representation of a raw angular sample as sent over the wire by the
 * DMU
 *
 * @struct ssdmu_raw_angular_sample
 */
struct ssdmu_raw_angular_sample {
    short dQ; /**< Rotation about x-axis */
    short dP; /**< Rotation about y-axis */
    short dR; /**< Rotation about z-axis */
    unsigned char status; /**< Status byte from DMU */
    unsigned char sequence; /**< Message sequence number */
} __attribute__((packed));

/**
 * Friendly raw angular sample type
 * @typedef ssdmu_raw_angular_sample_t
 */
typedef struct ssdmu_raw_angular_sample ssdmu_raw_angular_sample_t;

/**
 * Packed representation of a raw linear sample as sent over the wire by the
 * DMU
 *
 * @struct ssdmu_raw_linear_sample
 */
struct ssdmu_raw_linear_sample {
    short x; /**< Linear acceleration along x-axis */
    short y; /**< Linear acceleration along y-axis */
    short z; /**< Linear acceleration along z-axis */
    unsigned char status; /**< Status byte from DMU */
    unsigned char sequence; /**< Message sequence number */
} __attribute__((packed));

/**
 * Friendly raw linear sample type
 * @typedef ssdmu_raw_linear_sample_t
 */
typedef struct ssdmu_raw_linear_sample ssdmu_raw_linear_sample_t;

/**
 * Friendly sample from the DMU with linear accelerations and rotational
 * velocities represented in g and radians/sec respectively
 * @struct ssdmu_sample
 */
struct ssdmu_sample {
    double x; /**< Linear acceleration along x-axis as g */
    double y; /**< Linear acceleration along y-axis as g */
    double z; /**< Linear acceleration along z-axis as g */
    double dQ; /**< Rotation about x-axis as rads/sec */
    double dP; /**< Rotation about y-axis as rads/sec */
    double dR; /**< Rotation about z-axis as rads/sec */
};

/**
 * Friendly DMU sample type
 * @typedef ssdmu_sample_t
 */
typedef struct ssdmu_sample ssdmu_sample_t; 

/**
 * Opaque SSDMU instance for keeping track of the CAN bus, coordinate space
 * transform, etc.
 *
 * @struct ssdmu
 */
struct ssdmu {
    NTCAN_HANDLE handle;
    ssdmu_sample_t bias;
};

/**
 * Friendly SSDMU type
 * @typedef ssdmu_t
 */
typedef struct ssdmu ssdmu_t;

/**
 * Toggles streaming from a Silicon Sensing Dynamics Measurement Unit
 *
 * @function ssdmu_toggle_streaming
 * @param handle The NTCAN_HANDLE of the open and configured CAN bus on which 
 *      the DMU is located
 */
NTCAN_RESULT ssdmu_toggle_streaming(NTCAN_HANDLE handle);

/**
 * Enables or disables streaming. Tries to read a sample and toggles streaming
 * as needed based on whether the read times out
 *
 * @function ssdmu_eanble_streaming
 * @param handle The handle of the DMU's CAN bus
 * @param enable Whether to enable (non-zero) or disable (0) streaming from the DMU
 */
NTCAN_RESULT ssdmu_enable_streaming(NTCAN_HANDLE handle, int enable);

/**
 * Read a RAW sample from the DMU. This sample contains the raw signed 16-bit
 * integers as they are read off the bus.
 *
 * @function ssdmu_read_raw_sample
 * @param handle The handle of the DMU's CAN bus
 * @param angular A pointer to an ssdmu_raw_angular_sample_t struct to be
 *      populated with sample data from the DMU
 * @param linear A pointer to an ssdmu_raw_linear_sample_t struct to be
 *      populated with sample data from the DMU
 */
NTCAN_RESULT ssdmu_read_raw_sample(NTCAN_HANDLE handle, ssdmu_raw_angular_sample_t *angular, ssdmu_raw_linear_sample_t *linear);

/**
 * Scales a raw sample from the DMU into g and rads/sec. Using scaling
 * factors provided by Silicon Sensing
 *
 * @function ssdmu_scale_sample
 * @param raw_angular Pointer to the raw angular sample data to scale
 * @param raw_linear Pointer to the raw linear sample data to scale
 * @param sample Pointer to the friendly sample to populate with scaled data
 */
void ssdmu_scale_sample(ssdmu_raw_angular_sample_t *raw_angular, ssdmu_raw_linear_sample_t *raw_linear, ssdmu_sample_t *sample);

/**
 * Reads a friendly sample from the DMU. Most users should call this function 
 * to make their queries.
 *
 * @function ssdmu_read_sample
 * @param handle The handle of the DMU's CAN bus
 * @param sample Pointer to a sample struct to populate
 */
NTCAN_RESULT ssdmu_read_sample(NTCAN_HANDLE handle, ssdmu_sample_t *sample);

/**
 * Open a CAN handle and configure it for communication with the Silicon 
 * Sensing DMU.
 *
 * @function ssdmu_open
 * @param network Number of the CAN network to open on the local host
 * @param new_ssdmu Reference to a pointer to an ssdmu instance that will be
 *      allocated by ssdmu_open
 */
NTCAN_RESULT ssdmu_open(int network, ssdmu_t **new_ssdmu);

/**
 * Close an open CAN handle and free the associated ssdmu_t
 *
 * @function ssdmu_close
 * @param ssdmu The DMU to free
 */
NTCAN_RESULT ssdmu_close(ssdmu_t *ssdmu);

/**
 * Sample an open DMU, scale the sample to meaningful units, and correct the
 * sample based on the constant-error bias in the DMU
 *
 * @function ssdmu_sample
 * @param dmu The DMU to sample
 * @param sample Pointer to the sample structure to populate
 * @retrun NTCAN_SUCCESS on success, an NTCAN error code on failure
 */
NTCAN_RESULT ssdmu_sample(ssdmu_t *ssmdu, ssdmu_sample_t *sample);

/**
 * Find the pose of the robot (i.e., a rotation) given a sample from the DMU
 *
 * @function ssdmu_pose
 * @param sample The sample from which to calculate the pose
 * @param axis A 3-element double array to be overwritten with an axis of
 *      rotation
 * @param angle Point to a double to be overwritten the angle of rotation
 */
void ssdmu_pose(ssdmu_sample_t *sample, double *axis, double *angle);

/**
 * Subtract out the bias on each axis
 *
 * @function ssdmu_correct_sample
 * @param sample The sample to correct
 * @param bias The biases for each dimension
 */
void ssdmu_correct_sample(ssdmu_sample_t *sample, ssdmu_sample_t *bias);

double ssdmu_pitch(ssdmu_sample_t *sample);
double ssdmu_d_pitch(ssdmu_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif

