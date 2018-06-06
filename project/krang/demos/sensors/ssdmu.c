/** Author: martin, based on: jon olson
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <signal.h>
#include <ntcan.h>
#include <ssdmu/ssdmu.h>

#ifdef __cpluscplus
extern "C" {
#endif

#ifdef DEBUG
#define dprintf(f, args...) fprintf(stderr, f, ## args)
#else
#define dprintf(f, args...)
#endif

#define eprintf(f, args...) fprintf(stderr, f, ## args)

NTCAN_RESULT ssdmu_toggle_streaming(NTCAN_HANDLE handle) {
    CMSG canMsg;
    int len;
    int status;
    memset(&canMsg, 0, sizeof(canMsg));
    canMsg.id = 0x90;
    canMsg.len = 1;
    canMsg.data[0] = 0x0f;
    
    len = 1;
    status = canWrite(handle, &canMsg, &len, NULL);
    dprintf("ssdmu_toggle_streaming(): %d\n", status);
    
    return status;
}

NTCAN_RESULT ssdmu_read_raw_sample(NTCAN_HANDLE handle, ssdmu_raw_angular_sample_t *angular, ssdmu_raw_linear_sample_t *linear) {
    int status;
    int sequence = -1;
    int count = 0;
    CMSG canMsg;
    
    // This waits for two samples (one angular, one linear) from the DMU
    // It also requires that both samples have the same sequence number, so
    // we avoid working with data that is off-by-one
    while (count < 2) {
        int len = 1;
        status = canRead(handle, &canMsg, &len, NULL);
        dprintf("ssdmu_read_raw_sample(): %d (id=%x)\n", status, canMsg.id);
        if (status)
            return status;
        
        switch (canMsg.id) {
        case 0x40:
            memcpy(angular, canMsg.data, sizeof(ssdmu_raw_angular_sample_t));
            if (angular->sequence != sequence)
                count = 0;
            sequence = angular->sequence;
            count++;
            break;
        case 0x50:
            memcpy(linear, canMsg.data, sizeof(ssdmu_raw_linear_sample_t));
            if (linear->sequence != sequence)
                count = 0;
            sequence = linear->sequence;
            count++;
            break;
        default:
            // Not for us, move along
            break;
        }
    }
    
    return 0;
}

NTCAN_RESULT ssdmu_enable_streaming(NTCAN_HANDLE handle, int enable) {
    int status, enabled;
    ssdmu_raw_angular_sample_t angular;
    ssdmu_raw_linear_sample_t linear;

    status = ssdmu_read_raw_sample(handle, &angular, &linear);
    if (status == NTCAN_RX_TIMEOUT)
        enabled = 0;
    else if (status == NTCAN_SUCCESS)
        enabled = 1;
    else
        return status;

    dprintf("Enable streaming\n");

    // Are we wrong?
    if ((enabled && !enable) || (!enabled && enable)) {
        status = ssdmu_toggle_streaming(handle); // Toggle and pray :)
        if (status != NTCAN_SUCCESS)
            return status;
        return ssdmu_enable_streaming(handle, enable);
    }

    return NTCAN_SUCCESS;
}

#define ANGULAR_SCALE 0.008 / 180.0 * M_PI // Radians per second
#define LINEAR_SCALE 0.0003 // g (m/s^2)

void ssdmu_scale_sample(ssdmu_raw_angular_sample_t *raw_angular, ssdmu_raw_linear_sample_t *raw_linear, ssdmu_sample_t *sample) {
    sample->x = raw_linear->x * LINEAR_SCALE;
    sample->y = raw_linear->y * LINEAR_SCALE;
    sample->z = raw_linear->z * LINEAR_SCALE;
    sample->dP = raw_angular->dP * ANGULAR_SCALE;
    sample->dQ = raw_angular->dQ * ANGULAR_SCALE;
    sample->dR = raw_angular->dR * ANGULAR_SCALE;

    //printf("%f\t%f\t%f\t%f\t%f\t%f\n", sample->x,  sample->y,  sample->z,  sample->dQ,  sample->dP,  sample->dR);

}

NTCAN_RESULT ssdmu_read_sample(NTCAN_HANDLE handle, ssdmu_sample_t *sample) {
    ssdmu_raw_angular_sample_t angular;
    ssdmu_raw_linear_sample_t linear;
    
    NTCAN_RESULT status;
    status = ssdmu_read_raw_sample(handle, &angular, &linear);
    if (status != NTCAN_SUCCESS)
        return status;

    ssdmu_scale_sample(&angular, &linear, sample);
    
    return 0;
}

// And the DMU itself seems to exhibit the following bias.
// This shouldn't really be a structure full of constants, and we need a
// better method for correcting for bias. In particular, it would be nice if
// we could discover bias in real-time.

// Also, these are for the specific DMU on Golem -- we should have a better
// method for initializing these (like from a config file)
static ssdmu_sample_t ssdmu_bias = {
    .x = 0,
    .y = 0,
    .z = 0,
    .dP = 0.00443435541250234,
    .dQ = -0.192219813440485,
    .dR = -0.266295622593902
};

NTCAN_RESULT ssdmu_open(int network, ssdmu_t **new_ssdmu) {
    NTCAN_RESULT result;
    NTCAN_HANDLE handle;
    ssdmu_t *ssdmu = NULL;
    
    assert(new_ssdmu != NULL);
    
    result = canOpen(network,
        0, 
        100, 
        100, 
        50,
        50,
        &handle);
    if (result != NTCAN_SUCCESS)
        return result;
    
    // Set baudrate and add message IDs for SSDMU messages
    if ((result = canSetBaudrate(handle, NTCAN_BAUD_1000)) != NTCAN_SUCCESS) {
    	canClose(handle);  // fail!!!
    	return result;
    }

    if ((result = canIdAdd(handle, 0x40)) != NTCAN_SUCCESS) {
    	canClose(handle);  // fail!!!
    	return result;
    }

    if ((result = canIdAdd(handle, 0x50)) != NTCAN_SUCCESS) {
    	canClose(handle);  // fail!!!
    	return result;
    }

    if ((result = ssdmu_enable_streaming(handle, 1))) {
    	canClose(handle);  // fail!!!
    	return result;
    }
    
    // That went well. Now let's do the actual allocation
    ssdmu = (ssdmu_t *)malloc(sizeof(ssdmu_t));
    if (ssdmu == NULL) {
    	canClose(handle);
    	return result;	  // fail!!!
    }

    memset(ssdmu, 0, sizeof(ssdmu_t));
    ssdmu->handle = handle;
    ssdmu->bias = ssdmu_bias; // Copy in default biases
    
    *new_ssdmu = ssdmu;
    
    return NTCAN_SUCCESS;
    
}

NTCAN_RESULT ssdmu_close(ssdmu_t *ssdmu) {
    NTCAN_RESULT result;
    
    assert(ssdmu != NULL);
    result = canClose(ssdmu->handle);
    free(ssdmu); // We don't care what the result actually is -- we free here
                 // anyway.
    
    return result;
}

void ssdmu_correct_sample(ssdmu_sample_t *sample, ssdmu_sample_t *bias) {
    sample->x -= bias->x;
    sample->y -= bias->y;
    sample->z -= bias->z;
    sample->dP -= bias->dP;
    sample->dQ -= bias->dQ;
    sample->dR -= bias->dR;
}

NTCAN_RESULT ssdmu_sample(ssdmu_t *ssdmu, ssdmu_sample_t *sample) {
    int result;
    
    result = ssdmu_read_sample(ssdmu->handle, sample);
    if (result != NTCAN_SUCCESS)
        return result;
    
    ssdmu_correct_sample(sample, &ssdmu->bias);
    
    return NTCAN_SUCCESS;
}

// Simple vector functions -- we should use BLAS for this. Or do C++
static void vector3_cross(double *a, double *b, double *c) {
		int i;
    for (i = 1; i <= 3; i++) {
        int cur = i-1;
        int n1 = i % 3;
        int n2 = (i + 1) % 3;
        
        c[cur] = a[n1]*b[n2] - a[n2]*b[n1];
    }
}

static double vector3_mag(double *v) {
    double sum = 0;
    
		int i;
    for (i = 0; i < 3; i++)
        sum += v[i]*v[i];
    
    return sqrt(sum);
}

static void vector3_normalize(double *v_in, double *v_out) {
    double mag = vector3_mag(v_in);
    
		int i;
    for (i = 0; i < 3; i++)
        v_out[i] = v_in[i] / mag;
}

static double down[3] = { 0, 0, 1 };

// We've mounted the DMU at 45 degrees.
static const double csr = -.7853981634;

double ssdmu_pitch(ssdmu_sample_t *sample) {
    double x = sample->x;
    double y = sample->y;
    double newX;
		// double newY;

    newX = x*cos(csr) - y*sin(csr);
    // newY = x*sin(csr) + y*cos(csr);

    return atan2(newX, sample->z); 
}

double ssdmu_d_pitch(ssdmu_sample_t *sample) {
    double dP = sample->dP;
    double dQ = sample->dQ;
    // double newDP;
		double newDQ;

    // newDP = dP*cos(csr) - dQ*sin(csr);
    newDQ = dP*sin(csr) + dQ*cos(csr);

    return newDQ;
}

void ssdmu_pose(ssdmu_sample_t *sample, double *axis, double *angle) {
    double cross_axis[3];

    double x = sample->x;
    double y = sample->y;

    sample->x = x*cos(csr) - y*sin(csr);
    sample->y = y*cos(csr) + x*sin(csr);

    vector3_cross((double *)sample, down, cross_axis);
    vector3_normalize(cross_axis, axis);
    
    double sum[3];
    double diff[3];
    
    vector3_normalize((double *)sample, sum);
    memcpy(diff, sum, sizeof(double)*3);
    
    sum[2] += 1;
    diff[2] -= 1;
    
    double diff_mag = vector3_mag(diff);
    double sum_mag = vector3_mag(sum);
    
    *angle = 2*atan2(diff_mag, sum_mag);

    sample->x = x;
    sample->y = y;
}

#ifdef __cpluscplus
}
#endif

