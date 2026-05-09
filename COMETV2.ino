#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>

// ================== Pins ==================
constexpr uint8_t MAIN_PIN         = 2;
constexpr uint8_t DROGUE_PIN       = 3;
constexpr uint8_t BUZZER_PIN       = 6;
constexpr uint8_t MODE_SWITCH_PIN  = 21;

constexpr uint8_t RGB_RED_PIN      = 22;
constexpr uint8_t RGB_GREEN_PIN    = 24;
constexpr uint8_t RGB_BLUE_PIN     = 25;

constexpr uint8_t BATTERY_PIN      = A2;

// ================== I2C Addresses ==================
constexpr uint8_t IMU_ADDR = 0x6B;

// ================== Buzzer ==================
constexpr uint16_t BUZZER_FREQ_HZ = 4000;

// ================== Sensors ==================
Adafruit_MPL3115A2 mpl;
Adafruit_LSM6DSO32 imu;

bool mplOK = false;
bool imuOK = false;

float alt0 = 0.0f;

// ================== Battery Divider ==================
constexpr float ADC_REF_VOLTAGE = 3.3f;
constexpr float R_TOP = 430000.0f;
constexpr float R_BOTTOM = 100000.0f;
constexpr float BATTERY_DIVIDER_RATIO = ((R_TOP + R_BOTTOM) / R_BOTTOM);

// ================== RGB Mode ==================
int rgbMode = 0;

// ================== Helpers ==================
void beep(int times, int onMs = 100, int offMs = 100) {
  for (int i = 0; i < times; i++) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
    delay(onMs);
    noTone(BUZZER_PIN);
    delay(offMs);
  }
}

void setRGB(bool r, bool g, bool b) {
  digitalWrite(RGB_RED_PIN, r ? HIGH : LOW);
  digitalWrite(RGB_GREEN_PIN, g ? HIGH : LOW);
  digitalWrite(RGB_BLUE_PIN, b ? HIGH : LOW);
}

void updateRGBMode() {
  switch (rgbMode) {
    case 0: setRGB(1, 0, 0); break; // Red
    case 1: setRGB(0, 1, 0); break; // Green
    case 2: setRGB(0, 0, 1); break; // Blue
    case 3: setRGB(1, 1, 0); break; // Yellow
    case 4: setRGB(0, 1, 1); break; // Cyan
    case 5: setRGB(1, 0, 1); break; // Magenta
    case 6: setRGB(1, 1, 1); break; // White
    default: setRGB(0, 0, 0); break;
  }
}

void handleModeSwitch() {
  static bool buttonWasPressed = false;
  static unsigned long lastPressTime = 0;

  bool pressed = digitalRead(MODE_SWITCH_PIN) == LOW; // External pullup, pressed = LOW
  unsigned long now = millis();

  if (pressed && !buttonWasPressed && (now - lastPressTime > 150)) {
    buttonWasPressed = true;
    lastPressTime = now;

    rgbMode++;
    if (rgbMode > 6) {
      rgbMode = 0;
    }

    updateRGBMode();

    Serial.print("RGB Mode: ");
    Serial.println(rgbMode);

    beep(1, 40, 20);
  }

  if (!pressed) {
    buttonWasPressed = false;
  }
}

void rgbRainbowTest() {
  Serial.println("Testing RGB LED...");

  setRGB(1, 0, 0); Serial.println("RGB: Red");     delay(300);
  setRGB(0, 1, 0); Serial.println("RGB: Green");   delay(300);
  setRGB(0, 0, 1); Serial.println("RGB: Blue");    delay(300);
  setRGB(1, 1, 0); Serial.println("RGB: Yellow");  delay(300);
  setRGB(0, 1, 1); Serial.println("RGB: Cyan");    delay(300);
  setRGB(1, 0, 1); Serial.println("RGB: Magenta"); delay(300);
  setRGB(1, 1, 1); Serial.println("RGB: White");   delay(300);
  setRGB(0, 0, 0); Serial.println("RGB: Off");     delay(300);
}

void mosfetTest() {
  Serial.println("Testing MOSFET outputs...");
  Serial.println("WARNING: Make sure no charges/igniters are connected.");

  Serial.println("MAIN MOSFET ON");
  digitalWrite(MAIN_PIN, HIGH);
  beep(1, 100, 100);
  delay(1000);
  digitalWrite(MAIN_PIN, LOW);
  Serial.println("MAIN MOSFET OFF");

  delay(500);

  Serial.println("DROGUE MOSFET ON");
  digitalWrite(DROGUE_PIN, HIGH);
  beep(1, 100, 100);
  delay(1000);
  digitalWrite(DROGUE_PIN, LOW);
  Serial.println("DROGUE MOSFET OFF");

  delay(500);
}

float readBatteryVoltage(int samples = 5) {
  uint32_t sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(1);
  }

  float adcCounts = sum / (float)samples;
  float adcVoltage = (adcCounts / 4095.0f) * ADC_REF_VOLTAGE;
  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);

  uint8_t err = Wire.endTransmission(false);

  if (err != 0) return 0xFF;

  Wire.requestFrom(addr, (uint8_t)1);

  if (Wire.available()) return Wire.read();

  return 0xFF;
}

void scanI2C() {
  Serial.println();
  Serial.println("I2C Scan:");

  int found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);

      if (addr == 0x60) Serial.print("  MPL3115A2");
      if (addr == 0x6B) Serial.print("  LSM6DSO32 possible");
      if (addr == 0x7E) Serial.print("  Reserved / possible bus artifact");

      Serial.println();
      found++;
    }
  }

  if (found == 0) Serial.println("No I2C devices found.");

  Serial.println();
}

void setup() {
  pinMode(MAIN_PIN, OUTPUT);
  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // External 10k pullup already installed
  pinMode(MODE_SWITCH_PIN, INPUT);

  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  pinMode(BATTERY_PIN, INPUT);

  digitalWrite(MAIN_PIN, LOW);
  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  setRGB(0, 0, 0);

  analogReadResolution(12);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("================================");
  Serial.println("RP2040 FULL HARDWARE TEST");
  Serial.println("RGB + MODE SWITCH + BUZZER + MOSFETS + SENSORS");
  Serial.println("================================");

  beep(2, 100, 80);
  rgbRainbowTest();
  mosfetTest();

  Wire.begin();
  Wire.setClock(100000);
  delay(200);

  scanI2C();

  Serial.println("Checking MPL3115A2...");
  mplOK = mpl.begin();
  Serial.print("MPL3115A2 status: ");
  Serial.println(mplOK ? "OK" : "FAIL");

  if (mplOK) {
    delay(500);
    alt0 = mpl.getAltitude();
    Serial.print("Baseline altitude: ");
    Serial.println(alt0, 3);
  }

  Serial.println("Checking LSM6DSO32 at 0x6B...");

  uint8_t whoami = readRegister(IMU_ADDR, 0x0F);

  Serial.print("IMU WHO_AM_I: 0x");
  if (whoami < 16) Serial.print("0");
  Serial.println(whoami, HEX);

  imuOK = imu.begin_I2C(IMU_ADDR);

  Serial.print("LSM6DSO32 status: ");
  Serial.println(imuOK ? "OK" : "FAIL");

  if (imuOK) {
    imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  }

  if (mplOK && imuOK) {
    rgbMode = 1; // Green
    updateRGBMode();
    beep(3, 100, 80);
    Serial.println("All sensors OK");
  } else {
    rgbMode = 0; // Red
    updateRGBMode();
    beep(5, 100, 80);
    Serial.println("One or more sensors failed");
  }

  Serial.println("System Ready");
}

void loop() {
  handleModeSwitch();

  float batt = readBatteryVoltage();

  handleModeSwitch();

  Serial.print("BATT:");
  Serial.print(batt, 3);
  Serial.print(" V");

  if (mplOK) {
    float alt = mpl.getAltitude() - alt0;
    Serial.print(" | ALT:");
    Serial.print(alt, 3);
    Serial.print(" m");
  } else {
    Serial.print(" | ALT:FAIL");
  }

  handleModeSwitch();

  if (imuOK) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    imu.getEvent(&accel, &gyro, &temp);

    Serial.print(" | AX:");
    Serial.print(accel.acceleration.x, 3);
    Serial.print(" AY:");
    Serial.print(accel.acceleration.y, 3);
    Serial.print(" AZ:");
    Serial.print(accel.acceleration.z, 3);

    Serial.print(" | GX:");
    Serial.print(gyro.gyro.x, 3);
    Serial.print(" GY:");
    Serial.print(gyro.gyro.y, 3);
    Serial.print(" GZ:");
    Serial.print(gyro.gyro.z, 3);

    Serial.print(" | TEMP:");
    Serial.print(temp.temperature, 2);
    Serial.print(" C");
  } else {
    Serial.print(" | IMU:FAIL");
  }

  Serial.println();

  digitalWrite(MAIN_PIN, LOW);
  digitalWrite(DROGUE_PIN, LOW);

  handleModeSwitch();

  delay(10);
}
