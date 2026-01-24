#include "StepperTimerManager.h"

StepperTimerManager::StepperTimerManager() : _count(0)
{
    for (int i = 0; i < MAX_STEPPERS; ++i)
        _steppers[i] = nullptr;
}

StepperTimerManager& StepperTimerManager::Instance()
{
    static StepperTimerManager instance;
    return instance;
}

void StepperTimerManager::RegisterStepper(StepperMotorDriver* stepper)
{
    if (_count < MAX_STEPPERS)
        _steppers[_count++] = stepper;
}

void StepperTimerManager::UnregisterStepper(StepperMotorDriver* stepper)
{
    for (int i = 0; i < _count; ++i)
    {
        if (_steppers[i] == stepper)
        {
            _steppers[i] = _steppers[--_count];
            _steppers[_count] = nullptr;
            break;
        }
    }
}

void StepperTimerManager::OnTimerTick()
{
    for (int i = 0; i < _count; ++i)
    {
        _steppers[i]->OnTimerTick();
    }
}
