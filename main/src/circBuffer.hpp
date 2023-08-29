/** 
 * @file circBuffer.h
 * @brief Circular buffer for storing sensor data for the robot cup project
 * 
 * @cite P.J. Bones UCECE
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-07-22
 */


#ifndef CIRC_BUFFER_H
#define CIRC_BUFFER_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================
typedef struct {
    uint32_t size;
    uint32_t wirteIndex;
    uint32_t readIndex;
    int32_t* data;
} circBuffer_t;


// ===================================== Function Prototypes ==========================
void circBuffer_init(circBuffer_t *circBuffer, int32_t size);
int32_t circBuffer_read(circBuffer_t *circBuffer);
void circBuffer_write(circBuffer_t *circBuffer, int32_t value);
void circBuffer_free(circBuffer_t *circBuffer);
int32_t circBuffer_average(circBuffer_t *circBuffer);
void circBuffer_clear(circBuffer_t *circBuffer);

#endif // CIRC_BUFFER_H