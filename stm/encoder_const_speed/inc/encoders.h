

#ifndef ENC_H
#define ENC_H

#include "stm32f4xx.h"

volatile int16_t leftCount;
volatile int16_t rightCount;
volatile int16_t fwdCount;
volatile int16_t rotCount;
//distances
volatile float leftTotal;
volatile float rightTotal;
volatile float fwdTotal;
volatile float rotTotal;

// local variables
static volatile int16_t oldLeftEncoder;
static volatile int16_t oldRightEncoder;
static volatile int16_t leftEncoder;
static volatile int16_t rightEncoder;
static volatile int16_t encoderSum;
static volatile int16_t encoderDiff;

void encodersInit (void);
void encodersReset (void);
void encodersRead (void);

#endif


