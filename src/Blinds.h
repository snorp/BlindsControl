#ifndef Blinds_h_
#define Blinds_h_

#include <AccelStepper.h>

#define NUM_BLINDS_POSITIONS 3

class Blinds
{
  public:
    Blinds(uint8_t motorPin1, uint8_t motorPin2, uint8_t motorPin3,
           uint8_t motorPin4, long stepsPerRotation, uint32_t stateAddress, uint8_t switchPin = 0xff);

    uint8_t position() {
      return _position;
    }

    uint8_t angle() {
      return _angles[_position];
    }

    uint8_t* angles() {
      return _angles;
    }

    void setAngles(uint8_t* angles, size_t len) {
      memcpy(_angles, angles, min(len, NUM_BLINDS_POSITIONS) * sizeof(uint8_t));
    }

    uint8_t speed() {
      return _speed;
    }

    void setSpeed(uint8_t speed) {
      _speed = speed;
    }

    void movePosition(uint8_t position) {
      movePosition(position, _speed);
    }

    void movePosition(uint8_t position, uint8_t speed);

    void moveAngle(uint8_t position) {
      moveAngle(position, _speed);
    }

    void moveAngle(uint8_t position, uint8_t speed);

    void resetOrigin(void);
    void saveState(void);

  private:
    void restoreState(void);

    long _stepsPerRotation;
    uint32_t _stateAddress;
    uint8_t _speed;
    uint8_t _position; // current position, which is an index into the _angles array
    uint8_t _angles[NUM_BLINDS_POSITIONS];

    AccelStepper _stepper;
};

#endif // Blinds_h_
