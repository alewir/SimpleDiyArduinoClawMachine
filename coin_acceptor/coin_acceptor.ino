#include <Adafruit_PCD8544.h>  // include the Nokia 3310 screen library

/*
   Coin acceptor with COIN signal on pin 4.
   After a single coin inserted, there can be 1, 2 or 3 signals.
   When there are multiple signals from single coin insertion, then these come in 50ms intervals.
   Coin A gives 1 signal and adds 1 credit, coin B gives 2 signals and adds 2 credits, and coin C gives 3 signals and adds 5 credits.
   The cost of starting the game is 3 credits.
   Coins can be inserted all the time but no faster than every 220ms.
   When game is allowed to be started, then pin 2 is set to LOW state for 100ms.
   The credits get deducted if there is no game in progress and after machine is signalized to start.
   The feedback from machine about game finished is signalized by HIGH state received on pin 3 for 100ms.
*/

#define SCLK 7
#define DIN 8
#define DC 9
#define CS 11
#define RST 10

#define PIN_GAME_ALLOWED 2
#define PIN_GAME_FINISHED 3
#define PIN_COIN_SIGNAL 4

#define COIN_SIGNALS_1ZL 1
#define COIN_VAL_1ZL 1

#define COIN_SIGNALS_2ZL 2
#define COIN_VAL_2ZL 2

#define COIN_SIGNALS_5ZL 3
#define COIN_VAL_5ZL 5

#define GAME_START_COST 3

#define COINS_INTERVAL_MS 220
#define GAME_ALLOWED_SIGNAL_MS 100

Adafruit_PCD8544 display = Adafruit_PCD8544(SCLK, DIN, DC, CS, RST);  // create a display object

int singleCoinSignals = 0;
bool detectingNewSignal = false;
bool countingSingleCoinSignals = false;
bool gameAllowedSignalActive = false;
bool gameInProgress = false;
bool blinkOn = false;

int credits = 0;
int coins = 0;

unsigned long totalTime = 0;
unsigned long lastInterval = 0;

unsigned long firstSignalMs = 0;
unsigned long lastSignalMs = 0;
unsigned long gameAllowedSignalActivatedMs = 0;
unsigned long blinkStartMs = 0;

void setup() {
  display.begin();          // initialize the display
  display.setContrast(54);  // set the contrast
  display.clearDisplay();   // clear the display
  display.display();        // display the cleared image

  pinMode(PIN_COIN_SIGNAL, INPUT_PULLUP);  // set the COIN pin as an input

  pinMode(PIN_GAME_ALLOWED, OUTPUT);
  digitalWrite(PIN_GAME_ALLOWED, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_GAME_FINISHED), gameFinished, RISING);

  Serial.begin(9600);
  Serial.println("Coin acceptor initialized");
}

void gameFinished() {
  Serial.println("gameFinished");
  gameInProgress = false;
}

void loop() {
  if (blinkStartMs > 0 && millis() - blinkStartMs > 500) {
    blinkStartMs = 0;
    blinkOn = !blinkOn;
  } else if (blinkStartMs == 0) {
    blinkStartMs = millis();
  }

  if (digitalRead(PIN_COIN_SIGNAL) == LOW && !detectingNewSignal) {
    Serial.println("low");
    lastSignalMs = millis();
    detectingNewSignal = true;

    if (!countingSingleCoinSignals) {
      countingSingleCoinSignals = true;
      firstSignalMs = lastSignalMs;
    }
  }

  if (digitalRead(PIN_COIN_SIGNAL) == HIGH && detectingNewSignal) {
    Serial.println("  high");
    detectingNewSignal = false;
    singleCoinSignals++;
    lastInterval = millis() - lastSignalMs;
    Serial.print("interval:");
    Serial.println(lastInterval);
    Serial.print(" coinSignals:");
    Serial.println(singleCoinSignals);
  }

  if (countingSingleCoinSignals && !detectingNewSignal && (millis() - lastSignalMs >= COINS_INTERVAL_MS || singleCoinSignals >= COIN_SIGNALS_5ZL)) {
    Serial.print("countingEnd:");
    Serial.println(millis() - firstSignalMs);
    Serial.print(" lastSignal:");
    Serial.println(millis() - lastSignalMs);
    if (singleCoinSignals == COIN_SIGNALS_1ZL) {
      credits += COIN_VAL_1ZL;
    } else if (singleCoinSignals == COIN_SIGNALS_2ZL) {
      credits += COIN_VAL_2ZL;
    } else if (singleCoinSignals == COIN_SIGNALS_5ZL) {
      credits += COIN_VAL_5ZL;
    }

    coins++;
    Serial.print("coins:");
    Serial.println(coins);

    countingSingleCoinSignals = false;
    singleCoinSignals = 0;
    lastSignalMs = 0;
    firstSignalMs = 0;
    // TODO: Store credits value to eeprom
  }

  if (credits >= GAME_START_COST && !(gameAllowedSignalActive || gameInProgress)) {
    gameAllowedSignalActivatedMs = millis();
    gameAllowedSignalActive = true;

    digitalWrite(PIN_GAME_ALLOWED, HIGH);
  }

  if (gameAllowedSignalActive && millis() - gameAllowedSignalActivatedMs > GAME_ALLOWED_SIGNAL_MS) {
    gameAllowedSignalActivatedMs = 0;
    gameAllowedSignalActive = false;

    digitalWrite(PIN_GAME_ALLOWED, LOW);

    gameInProgress = true;  // disable this flag on interrupt from the machine on PIN_GAME_FINISHED
    credits -= GAME_START_COST;
    // TODO: Store credits value to eeprom
  }

  updateDisplay();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(BLACK);

  display.setCursor(0, 0);
  display.print("Credits: ");
  display.print(credits);

  display.setCursor(0, 10);
  int missing = GAME_START_COST - credits;
  if (missing > 0) {
    if (!gameInProgress) {
      display.print("Need: ");
      display.print(missing);
      display.print(" more");
    }
  } else {    
    display.print("(");
    display.print(credits / GAME_START_COST);
    display.print(" more games)");
  }

  display.setCursor(0, 30);
  if (credits < GAME_START_COST && !gameInProgress) {
    if (blinkOn) {
      display.print("INSERT COINS");
    } else {
      display.print("!!!!!! !!!!!");
    }
  } else if (gameAllowedSignalActive) {
    display.print("> ACTIVATING <");
  } else if (gameInProgress) {
    if (blinkOn) {
      display.setTextColor(WHITE, BLACK);
      display.print(" GAME ACTIVE ");
    } else {
      display.setTextColor(BLACK);
      display.print(" GAME ACTIVE ");
    }
  } else {
    display.setCursor(0, 40);
    display.print(totalTime);
    display.print(":");
    display.print(lastInterval);
    display.print(":");
    display.print(coins);
  }

  display.display();
}
