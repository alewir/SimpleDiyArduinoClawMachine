const int CLAW_GRASP_PIN = 49;
const int LED_PIN = 10;
const int GAME_ALLOWED_PIN = 19;
const int GAME_FINISHED_PIN = 20;

const int Z_STEPS_MAX = 1950;
const int ADDITIONAL_Z_STEPS_ON_INIT = 300;
const int STABILIZATION_TIME_MS = 2000;
const int BLINK_INTERVAL_MS = 500;
const int GAME_FINISHED_SIGNAL_MS = 100;
const int PULSE_WIDTH_US = 400;
const int ADD_INTERVAL_BETWEEN_STEPS_US = 200;

struct Pins {
  int motorStep[3];
  int motorDir[3];
  int motorEnabled[3];
  int min[3];
  int max[3];
  int joyXLeft;
  int joyXRight;
  int joyYForward;
  int joyYBackward;
  int joyButton;
};

struct Axes {
  int x;
  int y;
  int z;
};

const Axes AXIS = { 0, 1, 2 };
const Pins PINS = {
  { A0, A6, 46 },
  { A1, A7, 48 },
  { 38, A2, A8 },
  { 2, 14, 18 },
  { 3, 15, 18 },
  59,
  64,
  44,
  66,
  0
};

struct EndstopSwitches {
  int min[3];
  int max[3];
};

struct Joystick {
  int x;
  int y;
  int button;
};

struct Move {
  bool xLeft;
  bool xRight;
  bool yForward;
  bool yBackward;
};

struct State {
  bool starting;
  bool movingXY;
  bool dropping;
  bool lifting;
  bool homming;
  bool idling;
  bool standby;
};

struct Action {
  bool moveX;
  bool moveY;
  bool moveZ;
  bool liftClaw;
  bool dropClaw;
};

struct Controller {
  EndstopSwitches endstop;
  Joystick joystick;
  Move move;
};

Controller controller = {
  { { 0, 0, 1 },
    { 0, 0, 0 }
  },
  { 502, 502, 1 },
  { false, false, false, false }
};

State state = { false, false, false, false, false, false, false };
Action action = { false, false, false, false, false };
int z_steps = 0;
bool initialized = false;

bool newGameAllowed = false;
bool gameFinished = false;
unsigned long gameFinishedSignalMs = 0;
unsigned long blinkStartMs = 0;
bool blinkOn = false;

void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(PINS.motorStep[i], OUTPUT);
    pinMode(PINS.motorDir[i], OUTPUT);
    pinMode(PINS.motorEnabled[i], OUTPUT);
    digitalWrite(PINS.motorEnabled[i], HIGH);  // starting with disabled motors

    pinMode(PINS.min[i], INPUT_PULLUP);
    pinMode(PINS.max[i], INPUT_PULLUP);
  }

  pinMode(PINS.joyXLeft, INPUT_PULLUP);
  pinMode(PINS.joyXRight, INPUT_PULLUP);
  pinMode(PINS.joyYForward, INPUT_PULLUP);
  pinMode(PINS.joyYBackward, INPUT_PULLUP);
  pinMode(PINS.joyButton, INPUT_PULLUP);

  pinMode(CLAW_GRASP_PIN, OUTPUT);
  digitalWrite(CLAW_GRASP_PIN, LOW);  // starting with open claw

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // starting with lights off

  attachInterrupt(digitalPinToInterrupt(GAME_ALLOWED_PIN), gameAllowed, FALLING);

  pinMode(GAME_FINISHED_PIN, OUTPUT);
  digitalWrite(GAME_FINISHED_PIN, LOW);

  Serial.begin(9600);
  Serial.println("Claw machine initialized");

  state.lifting = true;
}

void gameAllowed() {
  if (initialized) {
    newGameAllowed = true;
  } else {
    Serial.println("Not initialized yet.");
  }
}

void enableLed(bool shouldBeEnabled) {
  digitalWrite(LED_PIN, shouldBeEnabled ? HIGH : LOW);
}

void clawGrasp(bool shouldBeGrasping) {
  digitalWrite(CLAW_GRASP_PIN, shouldBeGrasping ? HIGH : LOW);
}

void updateMotorEnabledState(bool stepNeeded, int motorEnabledPin) {
  if (stepNeeded) {
    digitalWrite(motorEnabledPin, LOW);  // Motor ON
  } else {
    digitalWrite(motorEnabledPin, HIGH);  // Motor OFF
  }
}

void updateMotorDirection(bool towardsMin, bool towardsMax, int minLimit, int maxLimit, bool& moving, int motorEnabledPin) {
  if (towardsMin && minLimit > 0) {
    digitalWrite(motorEnabledPin, LOW);  // LEFT or BACKWARD
    moving = true;
  } else if (towardsMax && maxLimit > 0) {
    digitalWrite(motorEnabledPin, HIGH);  // RIGHT or FORWARD
    moving = true;
  } else {
    moving = false;
  }
}

void readControllerState() {
  readEndstopSwitches();
  readJoystick();
}

void readEndstopSwitches() {
  controller.endstop.min[AXIS.x] = digitalRead(PINS.min[AXIS.x]);
  controller.endstop.max[AXIS.x] = digitalRead(PINS.max[AXIS.x]);
  controller.endstop.min[AXIS.y] = digitalRead(PINS.min[AXIS.y]);
  controller.endstop.max[AXIS.y] = digitalRead(PINS.max[AXIS.y]);
  controller.endstop.min[AXIS.z] = digitalRead(PINS.min[AXIS.z]);
}

void readJoystick() {
  controller.joystick.x = digitalRead(PINS.joyXLeft) == LOW ? -1 : (digitalRead(PINS.joyXRight) == LOW ? 1 : 0);
  controller.joystick.y = digitalRead(PINS.joyYForward) == LOW ? -1 : (digitalRead(PINS.joyYBackward) == LOW ? 1 : 0);
  controller.joystick.button = digitalRead(PINS.joyButton);
}

void movingClaw(Controller& controller, Action& action, int& z_steps, bool lift, bool drop) {
  stopXY(controller);
  action.liftClaw = lift;
  action.dropClaw = drop;
  z_steps++;
}

void stopXY(Controller& controller) {
  controller.move.xLeft = controller.move.xRight = controller.move.yForward = controller.move.yBackward = false;
}

void stoppingClaw(Action& action, int& z_steps) {
  action.liftClaw = false;
  action.dropClaw = false;
  z_steps = 0;
}

void handleStartingState(Controller& controller, State& state, Action& action, int& z_steps) {
  if (controller.endstop.min[AXIS.z] == 0 || z_steps > (Z_STEPS_MAX + 300)) {
    state.starting = false;
    stoppingClaw(action, z_steps);
    enableLed(true);
    state.movingXY = true;
  } else {
    movingClaw(controller, action, z_steps, true, false);
  }
}


void handleMovingXYState(Controller& controller, State& state) {
  if (controller.joystick.button == 0 && !(state.dropping || state.lifting || state.homming || state.idling || state.standby)) {
    state.movingXY = false;
    digitalWrite(LED_PIN, LOW);  // lights OFF
    state.dropping = true;
  } else {
    controller.move.xLeft = controller.joystick.x < 0;
    controller.move.xRight = controller.joystick.x > 0;
    controller.move.yForward = controller.joystick.y < 0;
    controller.move.yBackward = controller.joystick.y > 0;
  }
}

void handleDroppingState(Controller& controller, State& state, Action& action, int& z_steps) {
  if (z_steps > Z_STEPS_MAX) {
    state.dropping = false;
    stoppingClaw(action, z_steps);
    clawGrasp(true);
    state.lifting = true;
  } else {
    movingClaw(controller, action, z_steps, false, true);
  }
}

void handleLiftingState(Controller& controller, State& state, Action& action, int& z_steps) {
  if (controller.endstop.min[AXIS.z] == 0 || z_steps > Z_STEPS_MAX) {
    state.lifting = false;
    stoppingClaw(action, z_steps);
    state.homming = true;
  } else {
    movingClaw(controller, action, z_steps, true, false);
  }
}

void handleHommingState(Controller& controller, State& state, Action& action) {
  if (controller.endstop.min[AXIS.x] > 0 || controller.endstop.min[AXIS.y] > 0) {
    controller.move.xLeft = true;
    controller.move.yBackward = true;
    controller.move.xRight = controller.move.yForward = false;
  } else {
    state.homming = false;
    clawGrasp(false);
    delay(STABILIZATION_TIME_MS);
    state.idling = true;
  }
}

void handleIdlingState(Controller& controller, State& state, Action& action, int& z_steps) {
  if (z_steps > Z_STEPS_MAX) {
    state.idling = false;
    stoppingClaw(action, z_steps);
    state.standby = true;
    gameFinished = true;
  } else {
    movingClaw(controller, action, z_steps, false, true);
  }
}

void handleStandbyState(Controller& controller, State& state) {
  if (blinkStartMs > 0 && millis() - blinkStartMs > BLINK_INTERVAL_MS) {
    blinkStartMs = 0;
    blinkOn = !blinkOn;
  } else if (blinkStartMs == 0) {
    blinkStartMs = millis();
  }

  if (controller.joystick.button == 0 && newGameAllowed) {
    state.standby = false;
    state.starting = true;
    enableLed(false);
    newGameAllowed = false;
  } else {
    if (!initialized) {
      initialized = true;
    }

    if (gameFinished) {
      if (gameFinishedSignalMs == 0) {
        digitalWrite(GAME_FINISHED_PIN, HIGH);
        gameFinishedSignalMs = millis();
      } else if (millis() - gameFinishedSignalMs > GAME_FINISHED_SIGNAL_MS) {
        digitalWrite(GAME_FINISHED_PIN, LOW);
        gameFinishedSignalMs = 0;
        gameFinished = false;
      }
    }

    if (newGameAllowed) {
      enableLed(blinkOn);
    }
  }

  stopXY(controller);
}

void loop() {
  readControllerState();

  if (state.starting) {
    handleStartingState(controller, state, action, z_steps);
  } else if (state.movingXY) {
    handleMovingXYState(controller, state);
  } else if (state.dropping) {
    handleDroppingState(controller, state, action, z_steps);
  } else if (state.lifting) {
    handleLiftingState(controller, state, action, z_steps);
  } else if (state.homming) {
    handleHommingState(controller, state, action);
  } else if (state.idling) {
    handleIdlingState(controller, state, action, z_steps);
  } else if (state.standby) {
    handleStandbyState(controller, state);
  }

  preProcessAction(action, state);
  doTheStep();
}

void preProcessAction(Action& action, State state) {
  updateMotorDirection(controller.move.xLeft, controller.move.xRight, controller.endstop.min[AXIS.x], controller.endstop.max[AXIS.x], action.moveX, PINS.motorDir[AXIS.x]);
  updateMotorDirection(controller.move.yBackward, controller.move.yForward, controller.endstop.min[AXIS.y], controller.endstop.max[AXIS.y], action.moveY, PINS.motorDir[AXIS.y]);

  if (action.liftClaw) {
    digitalWrite(PINS.motorDir[AXIS.z], LOW);  // UP
    action.moveZ = true;
  } else if (action.dropClaw) {
    digitalWrite(PINS.motorDir[AXIS.z], HIGH);  // DOWN
    action.moveZ = true;
  } else {
    action.moveZ = false;
  }

  updateMotorEnabledState(action.moveX, PINS.motorEnabled[AXIS.x]);
  updateMotorEnabledState(action.moveY, PINS.motorEnabled[AXIS.y]);

  if (state.standby) {
    digitalWrite(PINS.motorEnabled[AXIS.z], HIGH);  // Z motor OFF
  } else if (action.moveZ || state.movingXY == true) {
    digitalWrite(PINS.motorEnabled[AXIS.z], LOW);  // Z motor ON
  }
}

void doTheStep() {
  delayMicroseconds(ADD_INTERVAL_BETWEEN_STEPS_US);

  if (action.moveX) {
    digitalWrite(PINS.motorStep[AXIS.x], HIGH);
  }
  if (action.moveY) {
    digitalWrite(PINS.motorStep[AXIS.y], HIGH);
  }
  if (action.moveZ) {
    digitalWrite(PINS.motorStep[AXIS.z], HIGH);
  }
  delayMicroseconds(PULSE_WIDTH_US);

  if (action.moveX) {
    digitalWrite(PINS.motorStep[AXIS.x], LOW);
  }
  if (action.moveY) {
    digitalWrite(PINS.motorStep[AXIS.y], LOW);
  }
  if (action.moveZ) {
    digitalWrite(PINS.motorStep[AXIS.z], LOW);
  }
  delayMicroseconds(PULSE_WIDTH_US);
}
