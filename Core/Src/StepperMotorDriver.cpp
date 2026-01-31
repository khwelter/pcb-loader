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
      _loadedDelayCounter(0),
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
    // Endschalter initialisieren
    for (int i = 0; i < 3; ++i)
    {
        _endswitches[i].port = nullptr;
        _endswitches[i].pin = 0;
        _endswitches[i].state = PositionState::POSITION_UNLOADED;
        _endswitches[i].prevState = PositionState::POSITION_UNLOADED;
        _endswitches[i].configured = false;
    }
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

void StepperMotorDriver::SetEndswitch(StepperPosition pos, GPIO_TypeDef* port, uint16_t pin)
{
    int idx = static_cast<int>(pos);
    if (idx >= 0 && idx < 3)
    {
        _endswitches[idx].port = port;
        _endswitches[idx].pin = pin;
        _endswitches[idx].configured = true;
    }
}

void StepperMotorDriver::SetPositionState(StepperPosition pos, PositionState state)
{
    int idx = static_cast<int>(pos);
    if (idx >= 0 && idx < 3)
    {
        _endswitches[idx].prevState = _endswitches[idx].state;
        _endswitches[idx].state = state;
    }
}

PositionState StepperMotorDriver::GetPositionState(StepperPosition pos) const
{
    int idx = static_cast<int>(pos);
    if (idx >= 0 && idx < 3)
    {
        return _endswitches[idx].state;
    }
    return PositionState::POSITION_UNLOADED;
}

bool StepperMotorDriver::IsPositionOccupied(StepperPosition pos) const
{
    int idx = static_cast<int>(pos);
    if (idx >= 0 && idx < 3 && _endswitches[idx].configured)
    {
        // Endschalter lesen (aktiv LOW angenommen)
        GPIO_PinState pinState = HAL_GPIO_ReadPin(_endswitches[idx].port, _endswitches[idx].pin);
        return (pinState == GPIO_PIN_RESET);  // LOW = belegt
    }
    return false;
}

void StepperMotorDriver::UpdateEndswitches()
{
    // Alle Endschalter aktualisieren
    for (int i = 0; i < 3; ++i)
    {
        if (_endswitches[i].configured)
        {
            _endswitches[i].prevState = _endswitches[i].state;

            GPIO_PinState pinState = HAL_GPIO_ReadPin(_endswitches[i].port, _endswitches[i].pin);
            if (pinState == GPIO_PIN_RESET)  // LOW = belegt
            {
                _endswitches[i].state = PositionState::POSITION_LOADED;
            }
            else
            {
                _endswitches[i].state = PositionState::POSITION_UNLOADED;
            }
        }
    }
}

void StepperMotorDriver::CheckLoadedCondition()
{
    // Prüfen ob irgendeine Position von UNLOADED auf LOADED gewechselt hat
    for (int i = 0; i < 3; ++i)
    {
        if (_endswitches[i].configured)
        {
            // Flanke UNLOADED -> LOADED erkennen
            if (_endswitches[i].prevState == PositionState::POSITION_UNLOADED &&
                _endswitches[i].state == PositionState::POSITION_LOADED)
            {
                // Motor läuft und Position wurde beladen
                if (_phase != MotionPhase::IDLE && _phase != MotionPhase::LOADED_DELAY)
                {
                    _phase = MotionPhase::LOADED_DELAY;
                    _loadedDelayCounter = 0;
                }
            }
        }
    }
}

void StepperMotorDriver::StartAbs(float targetPositionMm)
{
    // Prüfen ob alle konfigurierten Positionen UNLOADED sind
    bool canStart = true;
    for (int i = 0; i < 3; ++i)
    {
        if (_endswitches[i].configured)
        {
            if (_endswitches[i].state == PositionState::POSITION_LOADED)
            {
                canStart = false;
                break;
            }
        }
    }

    if (!canStart)
    {
        // Motor darf nicht starten - eine Position ist beladen
        return;
    }

    float targetSteps = MmToSteps(targetPositionMm);
    StartMotion(targetSteps);
}

void StepperMotorDriver::StartRel(float distanceMm)
{
    // Prüfen ob alle konfigurierten Positionen UNLOADED sind
    bool canStart = true;
    for (int i = 0; i < 3; ++i)
    {
        if (_endswitches[i].configured)
        {
            if (_endswitches[i].state == PositionState::POSITION_LOADED)
            {
                canStart = false;
                break;
            }
        }
    }

    if (!canStart)
    {
        // Motor darf nicht starten - eine Position ist beladen
        return;
    }

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
    _loadedDelayCounter = 0;
    EnableOutput(true);
}

void StepperMotorDriver::Stop()
{
    _enabled = false;
    _phase = MotionPhase::IDLE;
    _currentSpeed = 0.0f;
    _currentAccel = 0.0f;
    _loadedDelayCounter = 0;
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

    // Spezielle Phase: Nach LOADED 1 Sekunde weiterlaufen
    if (_phase == MotionPhase::LOADED_DELAY)
    {
        _loadedDelayCounter++;
        if (_loadedDelayCounter >= 1000)  // 1000 ms = 1 Sekunde
        {
            Stop();
            return;
        }
        // Konstante Geschwindigkeit beibehalten
        return;
    }

    // Verbleibende Distanz berechnen
    float remainingSteps = fabsf(_targetPositionSteps - _currentPositionSteps);

    // Prüfen ob Ziel erreicht
    if (remainingSteps < 0.5f)
    {
        Stop();
        _currentPositionSteps = _targetPositionSteps;
        return;
    }

    // === VERBESSERTE BREMSWEG-BERECHNUNG ===
    // Berechne vollständigen Bremsweg mit S-Kurve
    float v = _currentSpeed;
    float a = _acceleration;
    float j = _jerk;

    // Phase 1: Jerk-In beim Bremsen (Beschleunigung von 0 auf -a)
    float t1 = a / j;  // Zeit für Jerk-In
    float v1 = v - 0.5f * j * t1 * t1;  // Geschwindigkeit nach Jerk-In
    float s1 = v * t1 - (1.0f/6.0f) * j * t1 * t1 * t1;  // Weg während Jerk-In

    // Phase 2: Konstante Verzögerung (von v1 auf v2)
    float v2 = sqrtf(a * a / j);  // Geschwindigkeit bei Start Jerk-Out
    if (v2 > v1) v2 = v1;  // Sicherheit
    float t2 = (v1 - v2) / a;
    float s2 = v1 * t2 - 0.5f * a * t2 * t2;

    // Phase 3: Jerk-Out beim Bremsen (Beschleunigung von -a auf 0)
    float t3 = a / j;
    float s3 = v2 * t3 - 0.5f * a * t3 * t3 + (1.0f/6.0f) * j * t3 * t3 * t3;

    float totalBrakeDistance = s1 + s2 + s3;

    // Sicherheitsfaktor: 10% Reserve
    totalBrakeDistance *= 1.1f;

    // === PHASENÜBERGÄNGE ===
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

            // Prüfen ob wir sofort bremsen müssen
            if (remainingSteps <= totalBrakeDistance)
            {
                _phase = MotionPhase::JERK_IN_DECEL;
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
            if (remainingSteps <= totalBrakeDistance)
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
            if (remainingSteps <= totalBrakeDistance)
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
            if (remainingSteps <= totalBrakeDistance)
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
            float timeToZero = _currentSpeed / _acceleration;
            float distToZero = 0.5f * _currentSpeed * timeToZero;

            if (distToZero <= remainingSteps * 0.8f)
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
                _currentPositionSteps = _targetPositionSteps;
                Stop();
                return;
            }

            // Notbremse wenn Ziel fast erreicht
            if (remainingSteps < 2.0f)
            {
                _currentSpeed = 0.0f;
                _currentAccel = 0.0f;
                _currentPositionSteps = _targetPositionSteps;
                Stop();
                return;
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

    UpdateEndswitches();      // Endschalter aktualisieren
    CheckLoadedCondition();   // LOADED-Flanke prüfen
    UpdateMotionProfile();

    // Prüfen ob bereits am Ziel (doppelte Sicherheit)
    float remainingSteps = fabsf(_targetPositionSteps - _currentPositionSteps);
    if (remainingSteps < 0.1f)
    {
        _currentPositionSteps = _targetPositionSteps;
        Stop();
        return;
    }

    // Schritte generieren basierend auf aktueller Geschwindigkeit
    const float dt = 0.001f;  // 1 ms
    _stepAccumulator += _currentSpeed * dt;

    // Ganze Schritte ausführen
    while (_stepAccumulator >= 1.0f && remainingSteps > 0.5f)
    {
        StepPulse();
        _stepAccumulator -= 1.0f;

        // Position aktualisieren (Richtung beachten)
        if (_direction)
            _currentPositionSteps += 1.0f;
        else
            _currentPositionSteps -= 1.0f;

        // Restdistanz neu berechnen
        remainingSteps = fabsf(_targetPositionSteps - _currentPositionSteps);

        // Stopp wenn Ziel erreicht
        if (remainingSteps < 0.5f)
        {
            _currentPositionSteps = _targetPositionSteps;
            _stepAccumulator = 0.0f;
            Stop();
            return;
        }
    }
}

