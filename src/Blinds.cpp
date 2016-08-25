#include "Arduino.h"
#include "Blinds.h"
#include "EEPROMex.h"

#include <Debug.h>

#define STATE_MAGIC 0xba114b
#define ACCEL_RATE_STEPS 200
#define MAX_RPM 15

struct BlindsState {
  uint32_t magic;
  uint8_t angles[NUM_BLINDS_POSITIONS];
  uint8_t speed;
};

void
Blinds::restoreState(void) {
  BlindsState state;
  EEPROM.readBlock<BlindsState>(_stateAddress, state);
  if (state.magic != STATE_MAGIC) {
    return;
  }

  memcpy(_angles, state.angles, NUM_BLINDS_POSITIONS * sizeof(uint8_t));
  _speed = state.speed;
}

void
Blinds::saveState(void) {
  BlindsState state = { STATE_MAGIC, 0 };

  memcpy(state.angles, _angles, NUM_BLINDS_POSITIONS * sizeof(uint8_t));
  state.speed = _speed;
  EEPROM.updateBlock<BlindsState>(_stateAddress, state);
}

Blinds::Blinds(uint8_t motorPin1, uint8_t motorPin2, uint8_t motorPin3,
               uint8_t motorPin4, long stepsPerRotation, uint32_t stateAddress,
               uint8_t switchPin)
  : _stepsPerRotation(stepsPerRotation)
  , _stateAddress(stateAddress)
  , _speed(25)
  , _angles{ 0, 90, 180 }
  , _stepper(AccelStepper::HALF4WIRE, motorPin1, motorPin2, motorPin3, motorPin4)
{
  _stepper.setAcceleration(ACCEL_RATE_STEPS);
  if (switchPin != 0xff) {
    _stepper.setEnablePin(switchPin);
  }
  _position = 0;

  _stepper.disableOutputs();

  restoreState();
}

void
Blinds::movePosition(uint8_t pos, uint8_t speed)
{
  if (pos >= NUM_BLINDS_POSITIONS) {
    return;
  }

  uint8_t angle = _angles[pos];

  DEBUG("Moving to position %u", pos);

  moveAngle(angle, speed);
  _position = pos;
}

void
Blinds::moveAngle(uint8_t angle, uint8_t speed)
{
  DEBUG("Moving to angle %u, speed %u", angle, speed);

  _stepper.setMaxSpeed(map(speed, 1, 100, 1, (_stepsPerRotation / 60) * MAX_RPM));
  _stepper.enableOutputs();
  _stepper.runToNewPosition(map(angle, 0, 360, 0, _stepsPerRotation));
  _stepper.disableOutputs();

  DEBUG("Done moving");
}

void
Blinds::resetOrigin(void)
{
  DEBUG("Reset origin position");
  _stepper.setCurrentPosition(0);
}