#include "StepperMotorDriver.h"

StepperMotorDriver::StepperMotorDriver(GPIO_TypeDef* stepPort, uint16_t stepPin,
                                       GPIO_TypeDef* dirPort,  uint16_t dirPin,
                                       GPIO_TypeDef* enPort,   uint16_t enPin,
                                       uint16_t divider)
    : _stepPort(stepPort),
      _dirPort(dirPort),
      _enPort(enPort),
      _stepPin(stepPin),
      _dirPin(dirPin),
      _enPin(enPin),
      _divider(divider),
      _counter(0),
      _enabled(false),
      _direction(true)
{
}

void StepperMotorDriver::Init()
{
    // Grundzustand: alles deaktiviert
    HAL_GPIO_WritePin(_dirPort, _dirPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_stepPort, _stepPin, GPIO_PIN_RESET);
    EnableOutput(false);
}

void StepperMotorDriver::SetDirection(bool forward)
{
    _direction = forward;
    HAL_GPIO_WritePin(_dirPort, _dirPin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void StepperMotorDriver::SetDivider(uint16_t div)
{
    _divider = (div > 0) ? div : 1;
}

void StepperMotorDriver::EnableOutput(bool en)
{
    // Aktiv LOW: LOW = EIN, HIGH = AUS
    HAL_GPIO_WritePin(_enPort, _enPin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void StepperMotorDriver::Start()
{
    _counter = 0;
    _enabled = true;
    EnableOutput(true);   // einschalten (LOW)
}

void StepperMotorDriver::Stop()
{
    _enabled = false;
    EnableOutput(false);  // abschalten (HIGH)
}

void StepperMotorDriver::StepPulse()
{
    HAL_GPIO_WritePin(_stepPort, _stepPin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 60; ++i) { __NOP(); }
    HAL_GPIO_WritePin(_stepPort, _stepPin, GPIO_PIN_RESET);
}

void StepperMotorDriver::OnTimerTick()
{
    if (!_enabled)
        return;

    _counter++;
    if (_counter >= _divider)
    {
        _counter = 0;
        StepPulse();
    }
}
