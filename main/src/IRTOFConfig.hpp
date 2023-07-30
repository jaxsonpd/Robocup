/** 
 * @file IRTOFConfig.hpp
 * @brief Configuration file for the IR TOF sensors.
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @date 2023-07-30
 */


#ifndef IRTOFCONFIG_H
#define IRTOFCONFIG_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================
#define IRTOF_0 // Top IR TOF sensor
#define IRTOF_0_ADDR 0x30
#define IRTOF_0_TYPE 0 // 1.2m standard

#define IRTOF_1 // Bottom IR TOF sensor
#define IRTOF_1_ADDR 0x31
#define IRTOF_1_TYPE 0 // 1.2m standard

// ===================================== Function Prototypes ==========================


#endif // IRTOFCONFIG_H