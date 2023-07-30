/** 
 * @file IRConfig.h
 * @brief Configuration file for the IR triangulating sensors. This file should be included in any file that uses the IR sensors
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-15
 */


#ifndef IRCONFIG_H
#define IRCONFIG_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================
// ** IR triangulating sensor parameters **
#define IRTRI_20_150 2 // 20-150cm sensor type
#define IRTRI_20_150_RAWMIN 280 // 20-150cm sensor minium raw value (150cm)
#define IRTRI_20_150_RAWMAX 720 // 20-150cm sensor maximum raw value (20cm)

// ** IR triangulating sensor values **
// Number of IR triangulating sensors
#define IRTRI_NUM 2

// Parameters for IR triangulating sensor 0
#define IRTRI_0 // Top IR triangulating sensor
#define IRTRI_0_PIN A8
#define IRTRI_0_TYPE IRTRI_20_150

// Parameters for IR triangulating sensor 1
#define IRTRI_1 // Bottom IR triangulating sensor
#define IRTRI_1_PIN A9
#define IRTRI_1_TYPE IRTRI_20_150

// IR triangulating sensor constants
#define K_1 104
#define K_2 88887
#define K_3 -16

// IR trangulating sensor struct
typedef struct {
    uint8_t pin; // the pin the sensor is connected to
    uint8_t type; // 0: , 1:, 2: 20-150cm
} irTri_sensor_t;

// ===================================== Function Prototypes ==========================


#endif // IRCONFIG_H