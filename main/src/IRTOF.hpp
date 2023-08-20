/** 
 * @file IRTOF.h
 * @brief Header file for the IRTOF class
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-08-08
 */


#ifndef IRTOF_H
#define IRTOF_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include "VL53L0X.h"
#include "circBuffer.hpp"


// ===================================== Types/Constants ==============================
/**
 * @brief The IRTOF class used to interface with the VL53L0x Time of Flight sensor.
 * 
 */
class IRTOF {
    public:
        bool init(uint8_t address, uint8_t XSHUT, uint8_t bufferSize);
        void deInit(void);
        void update(void);
        int32_t getDistance(void);
    private:
        circBuffer_t* distanceBuffer;
        uint8_t bufferSize;
        uint8_t XSHUT;
        uint8_t address;
        VL53L0X sensor;
};

// ===================================== Function Prototypes ==========================


#endif // IRTOF_H