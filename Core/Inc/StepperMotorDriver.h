#ifndef STEPPER_MOTOR_DRIVER_H
#define STEPPER_MOTOR_DRIVER_H

#include "stm32f3xx_hal.h"

class StepperMotorDriver
{
public:
    StepperMotorDriver(GPIO_TypeDef* stepPort, uint16_t stepPin,
                       GPIO_TypeDef* dirPort,  uint16_t dirPin,
                       GPIO_TypeDef* enPort,   uint16_t enPin,
                       uint16_t divider = 1);

    void Init();
    void SetDirection(bool forward);
    void SetDivider(uint16_t div);

    void Start();          // aktiviert Enable, setzt _enabled = true
    void Stop();           // deaktiviert Enable
    void OnTimerTick();    // wird vom TimerManager aufgerufen

private:
    void StepPulse();
    void EnableOutput(bool en);  // intern: Pin‑Write

    GPIO_TypeDef* _stepPort;
    GPIO_TypeDef* _dirPort;
    GPIO_TypeDef* _enPort;

    uint16_t _stepPin;
    uint16_t _dirPin;
    uint16_t _enPin;
    uint16_t _divider;
    uint16_t _counter;

    bool _enabled;
    bool _direction;
};

#endif
