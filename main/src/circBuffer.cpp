/** 
 * @file circBuffer.c
 * @brief Circular buffer implementation for int32_t data
 * 
 * @cite P.J. Bones UCECE
 * @author Jack Duignan (JackpDuignan@gmail.com)
 * @date 2023-07-22
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "circBuffer.hpp"
// ===================================== Types/Constants ==============================


// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
/** 
 * @brief initializes a circular buffer
 * @param circBuffer pointer to the circular buffer to initialize
 * @param size size of the circular buffer
 * 
 */
void circBuffer_init(circBuffer_t *circBuffer, int32_t size) {
    circBuffer->size = size;
    circBuffer->wirteIndex = 0;
    circBuffer->readIndex = 0;
    circBuffer->data = (int32_t *) calloc(size, sizeof(int32_t)); // Use calloc to initialize to 0
}


/** 
 * @brief Read the next value from the circular buffer
 * @param circBuffer pointer to the circular buffer to read from
 *
 * @return the next value in the circular buffer 
 */
int32_t circBuffer_read(circBuffer_t *circBuffer) {
    int32_t value = circBuffer->data[circBuffer->readIndex];
    
    circBuffer->readIndex = (circBuffer->readIndex + 1) % circBuffer->size; // Ensure readIndex is always in range
    
    return value;
}


/** 
 * @brief Write a value to the circular buffer
 * @param circBuffer pointer to the circular buffer to write to
 * @param value value to write to the circular buffer
 * 
 */
void circBuffer_write(circBuffer_t *circBuffer, int32_t value) {
    circBuffer->data[circBuffer->wirteIndex] = value;
    
    circBuffer->wirteIndex = (circBuffer->wirteIndex + 1) % circBuffer->size; // Ensure wirteIndex is always in range

}


/** 
 * @brief Free the memory allocated to the circular buffer
 * @param circBuffer pointer to the circular buffer to free
 * 
 */
void circBuffer_free(circBuffer_t *circBuffer) {
    circBuffer->size = 0;
    circBuffer->wirteIndex = 0;
    circBuffer->readIndex = 0;
    free(circBuffer->data);
    circBuffer->data = NULL;
}


/** 
 * @brief Return the average of the values in the circular buffer
 * @param circBuffer pointer to the circular buffer to average
 * 
 * @return the average value
 */
int32_t circBuffer_average(circBuffer_t *circBuffer) {
    int32_t sum = 0;

    for (int i = 0; i < circBuffer->size; i++) {
        sum += circBuffer->data[i];
    }

    return sum / (int32_t) circBuffer->size;
}

