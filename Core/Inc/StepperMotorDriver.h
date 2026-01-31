#ifndef STEPPER_MOTOR_DRIVER_H
#define STEPPER_MOTOR_DRIVER_H

#include "stm32f3xx_hal.h"

enum class MotionPhase
{
    IDLE,
    JERK_IN,
    ACCEL,
    JERK_OUT_ACCEL,
    CONSTANT,
    JERK_IN_DECEL,
    DECEL,
    JERK_OUT_DECEL,
    LOADED_DELAY  // Neue Phase: 1 Sekunde Nachlauf
};

enum class StepperPosition
{
    PRELOAD_POS,
    LOAD_POS,
    UNLOAD_POS
};

enum class PositionState
{
    POSITION_UNLOADED,
    POSITION_LOADED
};

class StepperMotorDriver
{
public:
    StepperMotorDriver(GPIO_TypeDef* stepPort, uint16_t stepPin,
                       GPIO_TypeDef* dirPort,  uint16_t dirPin,
                       GPIO_TypeDef* enPort,   uint16_t enPin,
                       float stepsPerRev = 200.0f,
                       float mmPerRev = 40.0f);

    void Init();
    void SetDirection(bool forward);

    // Motion-Parameter (alle in steps/s bzw. steps/s² bzw. steps/s³)
    void SetMaxSpeed(float stepsPerSec);
    void SetAcceleration(float stepsPerSec2);
    void SetJerk(float stepsPerSec3);

    // Mechanik-Parameter
    void SetStepsPerRevolution(float steps);
    void SetMmPerRevolution(float mm);

    // Endschalter-Konfiguration
    void SetEndswitch(StepperPosition pos, GPIO_TypeDef* port, uint16_t pin);

    // Position-State setzen/abfragen
    void SetPositionState(StepperPosition pos, PositionState state);
    PositionState GetPositionState(StepperPosition pos) const;
    bool IsPositionOccupied(StepperPosition pos) const;

    // Bewegungssteuerung
    void StartAbs(float targetPositionMm);  // Absolute Zielposition
    void StartRel(float distanceMm);        // Relative Bewegung
    void Stop();                            // Sofortiger Stop

    void OnTimerTick();    // 1 ms Basis

    bool IsMoving() const { return _phase != MotionPhase::IDLE; }
    float GetCurrentPositionMm() const { return _currentPositionSteps / _stepsPerMm; }
    float GetTargetPositionMm() const { return _targetPositionSteps / _stepsPerMm; }
    void EnableOutput(bool en);

private:
    void StepPulse();
//    void EnableOutput(bool en);
    void UpdateMotionProfile();
    void StartMotion(float targetSteps);
    void UpdateEndswitches();
    void CheckLoadedCondition();  // Neu: prüft LOADED-Signal
    float MmToSteps(float mm) const { return mm * _stepsPerMm; }

    GPIO_TypeDef* _stepPort;
    GPIO_TypeDef* _dirPort;
    GPIO_TypeDef* _enPort;

    uint16_t _stepPin;
    uint16_t _dirPin;
    uint16_t _enPin;

    bool _enabled;
    bool _direction;

    // Endschalter (für 3 Positionen)
    struct Endswitch
    {
        GPIO_TypeDef* port;
        uint16_t pin;
        PositionState state;
        PositionState prevState;  // Neu: vorheriger Zustand
        bool configured;
    };

    Endswitch _endswitches[3];  // PRELOAD, LOAD, UNLOAD

    uint32_t _loadedDelayCounter;  // Neu: Zähler für 1 Sekunde

    // Mechanik
    float _stepsPerRev;
    float _mmPerRev;
    float _stepsPerMm;

    // Position
    float _currentPositionSteps;
    float _targetPositionSteps;

    // Motion Profile
    MotionPhase _phase;

    float _maxSpeed;          // steps/s
    float _acceleration;      // steps/s²
    float _jerk;              // steps/s³

    float _currentSpeed;      // steps/s
    float _currentAccel;      // steps/s²

    float _stepAccumulator;   // Bruchteil-Schritte
    uint32_t _phaseTime;      // Zeit in aktueller Phase (ms)
};

#endif
