#ifndef STEPPER_TIMER_MANAGER_H
#define STEPPER_TIMER_MANAGER_H

#include "StepperMotorDriver.h"

#define MAX_STEPPERS 8

class StepperTimerManager
{
public:
    static StepperTimerManager& Instance();

    void RegisterStepper(StepperMotorDriver* stepper);
    void UnregisterStepper(StepperMotorDriver* stepper);
    void OnTimerTick();   // Aufruf im externen Timer-ISR

private:
    StepperTimerManager();

    StepperMotorDriver* _steppers[MAX_STEPPERS];
    uint8_t _count;
};

#endif
