// ============================================================
//  IOT HEALTH & FITNESS MONITORING SYSTEM
//  ESP32 + MAX30102 + DS18B20 + MPU6050 + Firebase
//  FYP Project — iffatfyp
// ============================================================

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Sensor libraries
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050.h>

// ── YOUR CREDENTIALS (from config.h) ────────────────────────
// Create a file called config.h in the same folder and put:
// #define WIFI_SSID     "your_wifi"
// #define WIFI_PASSWORD "your_password"
// #define API_KEY       "your_firebase_api_key"
// #define DATABASE_URL  "your_firebase_database_url"
#include "config.h"

// ── PIN DEFINITIONS ─────────────────────────────────────────
#define DS18B20_PIN   4       // DS18B20 data pin → GPIO4
#define SDA_PIN       21      // I2C SDA → GPIO21
#define SCL_PIN       22      // I2C SCL → GPIO22

// ── FIREBASE OBJECTS ─────────────────────────────────────────
FirebaseData   fbdo;
FirebaseAuth   auth;
FirebaseConfig config;

// ── SENSOR OBJECTS ───────────────────────────────────────────
MAX30105        particleSensor;
OneWire         oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
MPU6050         mpu;

// ── HEART RATE VARIABLES ─────────────────────────────────────
const byte RATE_SIZE = 4;
byte   rates[RATE_SIZE];
byte   rateSpot = 0;
long   lastBeat = 0;
float  beatsPerMinute = 0;
int    beatAvg = 0;

// ── STEP COUNTER VARIABLES ───────────────────────────────────
int16_t ax, ay, az;
int16_t gx, gy, gz;
long    stepCount     = 0;
float   lastAccelMag  = 0;
bool    stepDetected  = false;
const float STEP_THRESHOLD = 1.2;   // tune this for your sensor

// ── TIMING ───────────────────────────────────────────────────
unsigned long lastSensorRead  = 0;
unsigned long lastFirebaseSend = 0;
unsigned long lastStatusUpdate = 0;
const unsigned long SENSOR_INTERVAL  = 100;    // read sensors every 100ms
const unsigned long FIREBASE_INTERVAL = 5000;  // send to Firebase every 5s
const unsigned long STATUS_INTERVAL   = 10000; // update device status every 10s

// ── THRESHOLDS (can be updated from Firebase) ─────────────────
float bpmMin    = 50.0;
float bpmMax    = 120.0;
float tempMin   = 35.5;
float tempMax   = 38.0;
int   stepsGoal = 8000;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Health Monitor Starting ===");

  // ── I2C ──
  Wire.begin(SDA_PIN, SCL_PIN);

  // ── MAX30102 ──
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30102 not found! Check wiring.");
  } else {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("✅ MAX30102 ready");
  }

  // ── DS18B20 ──
  tempSensor.begin();
  Serial.println("✅ DS18B20 ready");

  // ── MPU6050 ──
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 not found! Check wiring.");
  } else {
    Serial.println("✅ MPU6050 ready");
  }

  // ── WiFi ──
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected! IP: " + WiFi.localIP().toString());

  // ── Firebase ──
  config.api_key        = API_KEY;
  config.database_url   = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("✅ Firebase connected");

  // ── Set device online ──
  updateDeviceStatus(true);

  // ── Read thresholds from Firebase ──
  readThresholds();

  Serial.println("=== System Ready ===\n");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // ── Read sensors every 100ms ──
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readHeartRate();
    readSteps();
  }

  // ── Send to Firebase every 5 seconds ──
  if (now - lastFirebaseSend >= FIREBASE_INTERVAL) {
    lastFirebaseSend = now;
    if (Firebase.ready()) {
      sendHeartRate();
      sendTemperature();
      sendMotion();
      updateDailySummary();
      checkAlerts();
    }
  }

  // ── Update device status every 10 seconds ──
  if (now - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = now;
    updateDeviceStatus(true);
  }
}

// ============================================================
//  READ HEART RATE (MAX30102)
// ============================================================
void readHeartRate() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat   = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}

// ============================================================
//  READ STEPS (MPU6050)
// ============================================================
void readSteps() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw to g
  float axG = ax / 16384.0;
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;

  // Calculate acceleration magnitude
  float accelMag = sqrt(axG*axG + ayG*ayG + azG*azG);

  // Simple step detection
  if (accelMag > STEP_THRESHOLD && !stepDetected) {
    stepCount++;
    stepDetected = true;
  } else if (accelMag < 0.9) {
    stepDetected = false;
  }
  lastAccelMag = accelMag;
}

// ============================================================
//  GET ACTIVITY LEVEL
// ============================================================
String getActivity() {
  float accelMag = lastAccelMag;
  if (accelMag > 1.5) return "run";
  if (accelMag > STEP_THRESHOLD) return "walk";
  return "idle";
}

// ============================================================
//  GET TIMESTAMP
// ============================================================
String getTimestamp() {
  // Returns millis as string (replace with NTP for real time)
  unsigned long s = millis() / 1000;
  char buf[30];
  sprintf(buf, "uptime_%lu_sec", s);
  return String(buf);
}

// ============================================================
//  GET BPM STATUS
// ============================================================
String getBpmStatus(int bpm) {
  if (bpm > bpmMax) return "high";
  if (bpm < bpmMin) return "low";
  return "normal";
}

// ============================================================
//  GET TEMPERATURE STATUS
// ============================================================
String getTempStatus(float temp) {
  if (temp > tempMax) return "fever";
  if (temp < tempMin) return "low";
  return "normal";
}

// ============================================================
//  SEND HEART RATE TO FIREBASE
// ============================================================
void sendHeartRate() {
  if (beatAvg == 0) return; // no reading yet

  String path = "/health_monitor/sensor_data/heart_rate";
  FirebaseJson json;
  json.set("bpm",       beatAvg);
  json.set("status",    getBpmStatus(beatAvg));
  json.set("timestamp", getTimestamp());
  json.set("unix_time", (int)millis());

  if (Firebase.RTDB.pushJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("❤ Heart Rate sent: " + String(beatAvg) + " bpm");
  } else {
    Serial.println("❌ HR send failed: " + fbdo.errorReason());
  }
}

// ============================================================
//  SEND TEMPERATURE TO FIREBASE
// ============================================================
void sendTemperature() {
  tempSensor.requestTemperatures();
  float celsius    = tempSensor.getTempCByIndex(0);
  float fahrenheit = tempSensor.toFahrenheit(celsius);

  if (celsius == -127.0) {
    Serial.println("❌ DS18B20 read error");
    return;
  }

  String path = "/health_monitor/sensor_data/temperature";
  FirebaseJson json;
  json.set("celsius",    celsius);
  json.set("fahrenheit", fahrenheit);
  json.set("status",     getTempStatus(celsius));
  json.set("timestamp",  getTimestamp());
  json.set("unix_time",  (int)millis());

  if (Firebase.RTDB.pushJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("🌡 Temperature sent: " + String(celsius) + " °C");
  } else {
    Serial.println("❌ Temp send failed: " + fbdo.errorReason());
  }
}

// ============================================================
//  SEND MOTION/STEPS TO FIREBASE
// ============================================================
void sendMotion() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  String path = "/health_monitor/sensor_data/motion";
  FirebaseJson json;
  json.set("accel_x",   ax / 16384.0);
  json.set("accel_y",   ay / 16384.0);
  json.set("accel_z",   az / 16384.0);
  json.set("gyro_x",    gx / 131.0);
  json.set("gyro_y",    gy / 131.0);
  json.set("gyro_z",    gz / 131.0);
  json.set("steps",     (int)stepCount);
  json.set("activity",  getActivity());
  json.set("timestamp", getTimestamp());
  json.set("unix_time", (int)millis());

  if (Firebase.RTDB.pushJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("👟 Steps sent: " + String(stepCount));
  } else {
    Serial.println("❌ Motion send failed: " + fbdo.errorReason());
  }
}

// ============================================================
//  UPDATE DAILY SUMMARY
// ============================================================
void updateDailySummary() {
  // Estimate calories: steps * 0.04 kcal
  float calories = stepCount * 0.04;

  // Estimate active minutes from steps (every 100 steps ~ 1 min)
  int activeMin = stepCount / 100;

  String path = "/health_monitor/daily_summary/today";
  FirebaseJson json;
  json.set("avg_bpm",        beatAvg);
  json.set("total_steps",    (int)stepCount);
  json.set("calories_est",   calories);
  json.set("active_minutes", activeMin);
  json.set("date",           "today");

  Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json);
  Serial.println("📊 Daily summary updated");
}

// ============================================================
//  CHECK ALERTS
// ============================================================
void checkAlerts() {
  // Heart rate alert
  if (beatAvg > 0 && (beatAvg > bpmMax || beatAvg < bpmMin)) {
    String msg = beatAvg > bpmMax
      ? "High heart rate: " + String(beatAvg) + " bpm"
      : "Low heart rate: "  + String(beatAvg) + " bpm";

    FirebaseJson alertJson;
    alertJson.set("sensor",    "heart_rate");
    alertJson.set("severity",  beatAvg > 140 ? "critical" : "warning");
    alertJson.set("message",   msg);
    alertJson.set("value",     beatAvg);
    alertJson.set("timestamp", getTimestamp());
    alertJson.set("resolved",  false);
    Firebase.RTDB.pushJSON(&fbdo, "/health_monitor/alerts", &alertJson);
    Serial.println("⚠ ALERT: " + msg);
  }
}

// ============================================================
//  UPDATE DEVICE STATUS
// ============================================================
void updateDeviceStatus(bool online) {
  String path = "/health_monitor/device_status";
  FirebaseJson json;
  json.set("online",       online);
  json.set("ip_address",   WiFi.localIP().toString());
  json.set("wifi_rssi",    (int)WiFi.RSSI());
  json.set("firmware_ver", "1.0.0");
  json.set("last_seen",    getTimestamp());
  json.set("uptime_sec",   (int)(millis() / 1000));

  Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json);
}

// ============================================================
//  READ THRESHOLDS FROM FIREBASE
// ============================================================
void readThresholds() {
  String path = "/health_monitor/thresholds";
  if (Firebase.RTDB.getJSON(&fbdo, path.c_str())) {
    FirebaseJson &json = fbdo.jsonObject();
    FirebaseJsonData result;

    json.get(result, "bpm_min");    if (result.success) bpmMin    = result.floatValue;
    json.get(result, "bpm_max");    if (result.success) bpmMax    = result.floatValue;
    json.get(result, "temp_min");   if (result.success) tempMin   = result.floatValue;
    json.get(result, "temp_max");   if (result.success) tempMax   = result.floatValue;
    json.get(result, "steps_goal"); if (result.success) stepsGoal = result.intValue;

    Serial.println("✅ Thresholds loaded from Firebase");
  }
}
