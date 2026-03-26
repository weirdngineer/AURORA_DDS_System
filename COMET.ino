#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

// ================== Pins ==================
constexpr uint8_t STATUS_LED_PIN = 20;
constexpr uint8_t BUZZER_PIN     = 6;
constexpr uint8_t DROGUE_PIN     = 3;
constexpr uint8_t MAIN_PIN       = 2;
constexpr uint8_t BATTERY_PIN    = A2;   // CHANGE THIS if needed

// ================== Battery Divider ==================
constexpr float ADC_REF_VOLTAGE = 3.3f;
constexpr float R_TOP = 430000.0f;     // 430k
constexpr float R_BOTTOM = 100000.0f;  // 100k
constexpr float BATTERY_DIVIDER_RATIO = ((R_TOP + R_BOTTOM) / R_BOTTOM);  // 5.3 *10

// ================== Sensor ==================
Adafruit_MPL3115A2 mpl;
bool mplOK = false;

// ================== Altitude ==================
float alt0 = 0.0f;
float alt  = 0.0f;
float lastAlt = 0.0f;

// ================== Apogee Detection ==================
bool apogeeDetected = false;
bool drogueFired = false;
bool mainFired = false;

unsigned long apogeeTime = 0;

// ================== Heartbeat LED ==================
unsigned long lastBlink = 0;
bool ledState = false;
const unsigned long BLINK_INTERVAL = 100;

// ================== Helpers ==================
void beep(int times, int onMs = 100, int offMs = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    delay(offMs);
  }
}

float readBatteryVoltage(int samples = 10) {
  uint32_t sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }

  float adcCounts = sum / (float)samples;

  // RP2040 set to 12-bit below, so full scale = 4095
  float adcVoltage = (adcCounts / 4095.0f) * ADC_REF_VOLTAGE;
  float batteryVoltage = adcVoltage * BATTERY_DIVIDER_RATIO;

  return batteryVoltage;
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(MAIN_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(MAIN_PIN, LOW);

  analogReadResolution(12);  // RP2040 ADC = 12-bit

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("================================");
  Serial.println("RP2040 BARO + BATTERY TEST");
  Serial.print("Compiled: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  Serial.println("================================");

  beep(2, 60, 80);

  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  Serial.println("Checking MPL3115A2...");
  mplOK = mpl.begin();
  Serial.print("MPL3115A2 status: ");
  Serial.println(mplOK ? "OK" : "FAIL");

  if (!mplOK) {
    Serial.println("FATAL: MPL3115A2 missing.");

    while (1) {
      unsigned long now = millis();

      if (now - lastBlink >= BLINK_INTERVAL) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);
      }

      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }
  }

  delay(500);
  alt0 = mpl.getAltitude();
  lastAlt = 0.0f;

  Serial.print("Baseline altitude: ");
  Serial.println(alt0);

  float batt = readBatteryVoltage();
  Serial.print("Battery voltage: ");
  Serial.println(batt, 3);

  Serial.println("System Ready");
}

void loop() {
  unsigned long now = millis();

  // ================== Heartbeat ==================
  if (now - lastBlink >= BLINK_INTERVAL) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
  }

  // ================== Read Altitude ==================
  alt = mpl.getAltitude() - alt0;

  // ================== Read Battery ==================
  float batt = readBatteryVoltage();

  Serial.print("ALT: ");
  Serial.print(alt, 3);
  Serial.print(" m");

  Serial.print(" | BATT: ");
  Serial.print(batt, 3);
  Serial.println(" V");

  // ================== APOGEE DETECTION ==================
  if (!apogeeDetected) {
    if (alt < lastAlt - 3.0f) {
      apogeeDetected = true;
      apogeeTime = now;

      Serial.println("APOGEE DETECTED");
      beep(1, 1000, 80);
      digitalWrite(DROGUE_PIN, HIGH);
      delay(1000);
      digitalWrite(DROGUE_PIN, LOW);

      drogueFired = true;
    }
  }

  // ================== MAIN DEPLOY ==================
  if (apogeeDetected && !mainFired) {
    if (now - apogeeTime >= 5000) {
      Serial.println("MAIN DEPLOY");

      digitalWrite(MAIN_PIN, HIGH);
       beep(1, 1000, 80);
      delay(1000);
      digitalWrite(MAIN_PIN, LOW);

      mainFired = true;
    }
  }

  lastAlt = alt;

  delay(100);
}
