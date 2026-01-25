#include "StepperMotorDriver.h"
#include <cmath>

StepperMotorDriver::StepperMotorDriver(GPIO_TypeDef* stepPort, uint16_t stepPin,
                                       GPIO_TypeDef* dirPort,  uint16_t dirPin,
                                       GPIO_TypeDef* enPort,   uint16_t enPin,
                                       float stepsPerRev,
                                       float mmPerRev)
    : _stepPort(stepPort),
      _dirPort(dirPort),
      _enPort(enPort),
      _stepPin(stepPin),
      _dirPin(dirPin),
      _enPin(enPin),
      _enabled(false),
      _direction(true),
      _stepsPerRev(stepsPerRev),
      _mmPerRev(mmPerRev),
      _stepsPerMm(stepsPerRev / mmPerRev),
      _currentPositionSteps(0.0f),
      _targetPositionSteps(0.0f),
      _phase(MotionPhase::IDLE),
      _maxSpeed(1000.0f),
      _acceleration(500.0f),
      _jerk(2000.0f),
      _currentSpeed(0.0f),
      _currentAccel(0.0f),
      _stepAccumulator(0.0f),
      _phaseTime(0)
{
}

void StepperMotorDriver::Init()
{
    HAL_GPIO_WritePin(_dirPort, _dirPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_stepPort, _stepPin, GPIO_PIN_RESET);
    EnableOutput(false);
    _currentPositionSteps = 0.0f;
}

void StepperMotorDriver::SetDirection(bool forward)
{
    _direction = forward;
    HAL_GPIO_WritePin(_dirPort, _dirPin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void StepperMotorDriver::SetMaxSpeed(float stepsPerSec)
{
    _maxSpeed = stepsPerSec;
}

void StepperMotorDriver::SetAcceleration(float stepsPerSec2)
{
    _acceleration = stepsPerSec2;
}

void StepperMotorDriver::SetJerk(float stepsPerSec3)
{
    _jerk = stepsPerSec3;
}

void StepperMotorDriver::SetStepsPerRevolution(float steps)
{
    _stepsPerRev = steps;
    _stepsPerMm = _stepsPerRev / _mmPerRev;
}

void StepperMotorDriver::SetMmPerRevolution(float mm)
{
    _mmPerRev = mm;
    _stepsPerMm = _stepsPerRev / _mmPerRev;
}

void StepperMotorDriver::StartAbs(float targetPositionMm)
{
    float targetSteps = MmToSteps(targetPositionMm);
    StartMotion(targetSteps);
}

void StepperMotorDriver::StartRel(float distanceMm)
{
    float targetSteps = _currentPositionSteps + MmToSteps(distanceMm);
    StartMotion(targetSteps);
}

void StepperMotorDriver::StartMotion(float targetSteps)
{
    _targetPositionSteps = targetSteps;

    // Richtung bestimmen
    bool forward = (_targetPositionSteps >= _currentPositionSteps);
    SetDirection(forward);

    // Bewegung starten
    _enabled = true;
    _phase = MotionPhase::JERK_IN;
    _currentSpeed = 0.0f;
    _currentAccel = 0.0f;
    _stepAccumulator = 0.0f;
    _phaseTime = 0;
    EnableOutput(true);
}

void StepperMotorDriver::Stop()
{
    _enabled = false;
    _phase = MotionPhase::IDLE;
    _currentSpeed = 0.0f;
    _currentAccel = 0.0f;
    EnableOutput(false);
}

void StepperMotorDriver::EnableOutput(bool en)
{
    HAL_GPIO_WritePin(_enPort, _enPin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void StepperMotorDriver::StepPulse()
{
    HAL_GPIO_WritePin(_stepPort, _stepPin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 60; ++i) { __NOP(); }
    HAL_GPIO_WritePin(_stepPort, _stepPin, GPIO_PIN_RESET);
}

void StepperMotorDriver::UpdateMotionProfile()
{
    const float dt = 0.001f;  // 1 ms in Sekunden
    _phaseTime++;

    // Verbleibende Distanz berechnen
    float remainingSteps = fabsf(_targetPositionSteps - _currentPositionSteps);

    // Prüfen ob Ziel erreicht
    if (remainingSteps < 0.5f)
    {
        Stop();
        _currentPositionSteps = _targetPositionSteps;
        return;
    }

    switch (_phase)
    {
        case MotionPhase::JERK_IN:
        {
            // Beschleunigung steigt linear
            _currentAccel += _jerk * dt;
            if (_currentAccel >= _acceleration)
            {
                _currentAccel = _acceleration;
                _phase = MotionPhase::ACCEL;
                _phaseTime = 0;
            }
            break;
        }

        case MotionPhase::ACCEL:
        {
            // Konstante Beschleunigung
            _currentAccel = _acceleration;

            // Prüfen ob wir Maximalgeschwindigkeit erreichen
            float speedDiff = _maxSpeed - _currentSpeed;
            float accelNeeded = (_currentAccel * _currentAccel) / (2.0f * _jerk);

            if (speedDiff <= accelNeeded)
            {
                _phase = MotionPhase::JERK_OUT_ACCEL;
                _phaseTime = 0;
            }

            // Prüfen ob wir bremsen müssen
            float stepsToStop = (_currentSpeed * _currentSpeed) / (2.0f * _acceleration);
            if (remainingSteps <= stepsToStop * 1.2f)  // 20% Sicherheit
            {
                _phase = MotionPhase::JERK_IN_DECEL;
                _phaseTime = 0;
            }
            break;
        }

        case MotionPhase::JERK_OUT_ACCEL:
        {
            // Beschleunigung fällt ab
            _currentAccel -= _jerk * dt;
            if (_currentAccel <= 0.0f || _currentSpeed >= _maxSpeed)
            {
                _currentAccel = 0.0f;
                _currentSpeed = _maxSpeed;
                _phase = MotionPhase::CONSTANT;
                _phaseTime = 0;
            }

            // Prüfen ob wir bremsen müssen
            float stopsToStop = (_currentSpeed * _currentSpeed) / (2.0f * _acceleration);
            if (remainingSteps <= stopsToStop * 1.2f)
            {
                _phase = MotionPhase::JERK_IN_DECEL;
                _phaseTime = 0;
            }
            break;
        }

        case MotionPhase::CONSTANT:
        {
            // Konstante Geschwindigkeit
            _currentAccel = 0.0f;
            _currentSpeed = _maxSpeed;

            // Prüfen ob wir bremsen müssen
            float stepsToStop = (_currentSpeed * _currentSpeed) / (2.0f * _acceleration);
            if (remainingSteps <= stepsToStop * 1.2f)
            {
                _phase = MotionPhase::JERK_IN_DECEL;
                _phaseTime = 0;
            }
            break;
        }

        case MotionPhase::JERK_IN_DECEL:
        {
            // Bremsbeschleunigung steigt
            _currentAccel -= _jerk * dt;
            if (_currentAccel <= -_acceleration)
            {
                _currentAccel = -_acceleration;
                _phase = MotionPhase::DECEL;
                _phaseTime = 0;
            }
            break;
        }

        case MotionPhase::DECEL:
        {
            // Konstantes Bremsen
            _currentAccel = -_acceleration;

            // Prüfen ob wir Jerk-Out starten müssen
            if (_currentSpeed <= _acceleration / 2.0f)
            {
                _phase = MotionPhase::JERK_OUT_DECEL;
                _phaseTime = 0;
            }
            break;
        }

        case MotionPhase::JERK_OUT_DECEL:
        {
            // Bremsbeschleunigung fällt ab
            _currentAccel += _jerk * dt;
            if (_currentAccel >= 0.0f || _currentSpeed <= 0.0f)
            {
                _currentAccel = 0.0f;
                _currentSpeed = 0.0f;
                Stop();
                _currentPositionSteps = _targetPositionSteps;
            }
            break;
        }

        default:
            break;
    }

    // Geschwindigkeit aktualisieren
    _currentSpeed += _currentAccel * dt;

    // Begrenzung
    if (_currentSpeed > _maxSpeed)
        _currentSpeed = _maxSpeed;
    if (_currentSpeed < 0.0f)
        _currentSpeed = 0.0f;
}

void StepperMotorDriver::OnTimerTick()
{
    if (!_enabled || _phase == MotionPhase::IDLE)
        return;

    UpdateMotionProfile();

    // Schritte generieren basierend auf aktueller Geschwindigkeit
    const float dt = 0.001f;  // 1 ms
    _stepAccumulator += _currentSpeed * dt;

    // Ganze Schritte ausführen
    while (_stepAccumulator >= 1.0f)
    {
        StepPulse();
        _stepAccumulator -= 1.0f;

        // Position aktualisieren (Richtung beachten)
        if (_direction)
            _currentPositionSteps += 1.0f;
        else
            _currentPositionSteps -= 1.0f;
    }
}
