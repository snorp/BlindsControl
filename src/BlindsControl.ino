#include <EEPROMex.h>
#include <Node.h>

#define ENCRYPT_KEY "ak2950sjzcoajsd9"

// #define NODE_DEBUG
#include <Debug.h>

#include "Blinds.h"

#define LED_PIN           9
#define BUTTON_PIN        3
#define MOTOR_PINS        15, 17, 16, 18
#define MOTOR_STEPS_PER_ROTATION 4096

#define BATTERY_PIN       A0
#define BATTERY_MIN_MV    1400 * 4
#define BATTERY_MAX_MV    1800 * 4
#define BATTERY_SENSE_RATIO float(2000000 + 2700000) / 2000000.0

#define BLINDS_STATE_EEPROM_ADDRESS 0 // Where to store blinds state

Node node;
Blinds blinds(MOTOR_PINS, MOTOR_STEPS_PER_ROTATION, BLINDS_STATE_EEPROM_ADDRESS);

uint8_t lastPosition = 0;
int8_t direction = 1;
bool buttonPushed = false;
byte lastButtonVal = 1;

void flashLED(void)
{
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}

void buttonInterruptHandler()
{
  byte val = digitalRead(BUTTON_PIN);
  if (val == lastButtonVal) {
    return;
  }

  lastButtonVal = val;
  if (!val) {
    // Button is down
    buttonPushed = true;
  }
}

static void sendPosition(uint8_t to, bool isReply)
{
  node.sendMessage(to, CMD_OP_BLINDS_POSITION,
                   blinds.position(), 0,
                   isReply ? CMD_FLAG_REPLY : 0);
}

static void moveNextStep(uint8_t from = GATEWAY_ADDRESS, bool sendResult = true) {
  if ((lastPosition + direction) == NUM_BLINDS_POSITIONS) {
    direction = -1;
  } else if ((lastPosition + direction) < 0) {
    direction = 1;
  }
  lastPosition += direction;
  blinds.movePosition(lastPosition);
  if (sendResult) {
    sendPosition(from, false);
  }
}

static void sendAngle(uint8_t to, bool isReply)
{
  node.sendMessage(to, CMD_OP_BLINDS_ANGLE,
                   blinds.angle(), 0,
                   isReply ? CMD_FLAG_REPLY : 0);
}

static void sendSpeed(uint8_t to)
{
  node.sendMessage(to, CMD_OP_BLINDS_SPEED,
                   blinds.speed());
}

static void sendAngles(uint8_t to)
{
#if NUM_BLINDS_POSITIONS == 3
  uint8_t* angles = blinds.angles();
  uint16_t arg1 = (uint16_t(angles[0]) << 8) | uint16_t(angles[1]);
  node.sendMessage(to, CMD_OP_BLINDS_ANGLES, arg1, angles[2]);
#else
#error Write some code
#endif
}

void handleMessage(Message& msg) {
  switch (msg.cmd.op) {
    case CMD_OP_POLL:
      DEBUG("Polling...");
      node.sendInfo(msg.from);
      sendPosition(msg.from, false);
      sendAngle(msg.from, false);
      sendSpeed(msg.from);
      sendAngles(msg.from);

      // We're done sending stuff
      node.sendMessage(msg.from, CMD_OP_POLL, 0, 0, CMD_FLAG_REPLY);
      DEBUG("Done polling");
      break;
    case CMD_OP_BLINDS_POSITION:
      blinds.movePosition(msg.cmd.arg1);
      if (!msg.isBroadcast()) {
        sendPosition(msg.from, true);
        node.sendBatteryLevel(msg.from);
      }
      break;
    case CMD_OP_BLINDS_ANGLE:
      blinds.moveAngle(msg.cmd.arg1);
      if (!msg.isBroadcast()) {
        sendAngle(msg.from, true);
        node.sendBatteryLevel(msg.from);
      }
      break;
    case CMD_OP_BLINDS_STEP:
      moveNextStep(msg.from, !msg.isBroadcast());
      break;
    case CMD_OP_BLINDS_ANGLES:
#if NUM_BLINDS_POSITIONS == 3
      uint8_t angles[NUM_BLINDS_POSITIONS];
      angles[0] = (msg.cmd.arg1 >> 8) & 0xff;
      angles[1] = msg.cmd.arg1 & 0xff;
      angles[2] = msg.cmd.arg2;
      blinds.setAngles(angles, NUM_BLINDS_POSITIONS);

      // The angle for the current position may have changed, so go to the new one if so
      blinds.movePosition(blinds.position());
      blinds.saveState();

      DEBUG("Set blinds angles to %d, %d, %d",
            angles[0],
            angles[1],
            angles[2]);
#else
#error Write some code
#endif
      break;
    case CMD_OP_BLINDS_SPEED:
      blinds.setSpeed(msg.cmd.arg1);
      blinds.saveState();
      break;
    default:
      break;
  }
}

void setup() {
  DEBUG_INIT(9600);

  node.setup(DEFAULT_ADDRESS,               // default address, real address may come from EEPROM
             DEFAULT_NETWORK_ID,            // network ID
             DEFAULT_NODE_SETTINGS_ADDRESS, // EEPPROM start address for storing settings
             ENCRYPT_KEY,                   // encryption key
             true,                          // has flash
             kTiltBlinds);     // device class
  node.setBatteryParams(BATTERY_PIN, BATTERY_MAX_MV, BATTERY_MIN_MV, BATTERY_SENSE_RATIO);
  node.onMessage(handleMessage);

  DEBUG("Blinds Controller Init, address=%u", node.address());

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterruptHandler, CHANGE);

  DEBUG("Setup complete");

  // Ping the gateway on startup to let it know we're here
  node.sendMessage(GATEWAY_ADDRESS, CMD_OP_PING);
}

void loop() {
  if (buttonPushed) {
    buttonPushed = false;
    flashLED();
    delay(200);
    flashLED();
    blinds.resetOrigin();
  }

  node.run();
}
