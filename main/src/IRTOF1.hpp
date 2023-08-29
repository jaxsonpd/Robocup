/** 
 * @file IRTOF1.hpp
 * @brief Header file for the IRTOF class which is used to interface with the VL53L1x sensor
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-08-20
 */


#ifndef IRTOF1_H
#define IRTOF1_H        


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include "VL53L1X.h"
#include "circBuffer.hpp"

// ===================================== Types/Constants ==============================
/**
 * @brief The IRTOF class used to interface with the VL53L1x Time of Flight sensor.
 * 
 */
class IRTOF1 {
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
        VL53L1X sensor;
};

// ===================================== Function Prototypes ==========================


#endif // IRTOF1_H      