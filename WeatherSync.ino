// ════════════════════════════════════════════════════════════════════
//  UNIFIED WEATHER STATION — ESP32-S3
//  TWO-TIER SAMPLING / PUBLISH REVISION
//
//  Sensors:
//    1. Wind Direction Sensor        (4–20 mA current loop, GPIO4)
//    2. VEML6070  UV Sensor          (I2C)
//    3. BMP390    Pressure/Temp/Alt  (I2C)
//    4. DFRobot   Tipping Bucket     (I2C)
//    5. SHT85     Temp/Humidity      (I2C)
//    6. Anemometer 4–20 mA           (Analog, GPIO7)
//
//  SAMPLING / PUBLISH STRATEGY (this revision):
//    • FAST tier  (every FAST_SAMPLE_MS,  default 3s):
//        wind speed, wind direction, rain cumulative poll.
//        Wind is gusty — sampling every 30s+ throws away real peaks,
//        so wind speed/direction are accumulated frequently and
//        reduced to avg + gust at publish time.
//    • SLOW tier  (every SLOW_SAMPLE_MS, default 30s):
//        pressure, temp, humidity, UV. These drift slowly, so
//        frequent sampling isn't needed — accumulated for averaging.
//    • MQTT publish (every MQTT_PUBLISH_MS, default 30s):
//        latest instantaneous readings, for the app's live dashboard.
//        Retained, small payload, no averaging — "what's happening now".
//    • SUPABASE publish (every SUPABASE_PUBLISH_MS, default 5 min):
//        averaged/aggregated values since the last Supabase publish —
//        this is the time-series the app's forecasting/trend logic
//        (pressure trend, dew point, rain-rate windows) runs on.
//        Accumulators reset after each Supabase publish.
//
//  IMPORTANT — SUPABASE RPC SIGNATURE CHANGED:
//    The log_reading RPC now needs to accept the AGGREGATE field set
//    below (pressure_hpa_avg, wind_speed_ms_gust, rain_mm_interval,
//    etc.) instead of the older single-sample field set. Update your
//    Supabase function signature to match p_* params in
//    postAggregateToSupabase() before flashing, or the RPC calls will
//    fail with a parameter-mismatch error.
//
//  REQUIRED LIBRARIES (Arduino Library Manager):
//    • PubSubClient           (Nick O'Leary)
//    • ArduinoJson             (Benoit Blanchon) — v6.x API used below
//    • DFRobot_RainfallSensor  (DFRobot) — I2C class used
//    (WiFi, WiFiClientSecure, HTTPClient ship with the ESP32 core)
// ════════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────────
//  WIRING SUMMARY  (ESP32-S3) — unchanged from previous revision
//
//  ── SHARED I2C BUS (SDA=GPIO8, SCL=GPIO9) ──────────────────────
//  Pull-ups: GPIO8 — 10kΩ — 3V3 | GPIO9 — 10kΩ — 3V3
//  VEML6070 (UV)        VDD→3V3  GND→GND  SDA→GPIO8  SCL→GPIO9
//  BMP390 (Pressure)    VCC→3V3  GND→GND  SDA→GPIO8  SCL→GPIO9
//  SHT85 (Temp/RH)      VDD→3V3  GND→GND  SDA→GPIO8  SCL→GPIO9
//  DFRobot Tipping Bkt  VCC→3V3  GND→GND  SDA→GPIO8  SCL→GPIO9  (0x1D)
//
//  ── Anemometer 4–20 mA — WIND SPEED (Analog) ─────────────────────
//    Signal → GPIO7 (ADC1_CH6), 156Ω shunt → GND
//
//  ── Wind Direction Sensor 4–20 mA (Analog) ────────────────────────
//    Signal → GPIO4 (ADC1_CH3), 150Ω shunt → GND
//
//  Enclosures (per your build):
//    • BMP390 + SHT85 → vented PETG radiation shield
//    • VEML6070       → sealed transparent weatherproof box, ePTFE
//                        membrane for pressure/humidity equalization
//    • ESP32 + power  → sealed opaque weatherproof box, ePTFE membrane
//    • Wind sensors   → mast-mounted, IP-rated, no shield needed
// ────────────────────────────────────────────────────────────────────

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "SHT85.h"
#include "DFRobot_RainfallSensor.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ════════════════════════════════════════════════════════════════════
//  NETWORK / BACKEND CONFIG — fill these in before flashing
// ════════════════════════════════════════════════════════════════════
#define WIFI_SSID         "IEEE_ITWEATHERSTATION"
#define WIFI_PASSWORD     "Shourya@0210"
#define STATION_ID        "ws-001"

#define HIVEMQ_HOST       ""
#define HIVEMQ_PORT       
#define HIVEMQ_USERNAME   ""
#define HIVEMQ_PASSWORD   ""

// Supabase — same project as the app's EXPO_PUBLIC_SUPABASE_URL /
// EXPO_PUBLIC_SUPABASE_ANON_KEY values.
#define SUPABASE_URL      ""
#define SUPABASE_ANON_KEY ""

// ════════════════════════════════════════════════════════════════════
//  TIMING — the two-tier schedule
// ════════════════════════════════════════════════════════════════════
#define FAST_SAMPLE_MS       3000UL      // wind + rain poll
#define SLOW_SAMPLE_MS       30000UL     // pressure/temp/humidity/UV poll
#define MQTT_PUBLISH_MS      30000UL     // live snapshot to app
#define SUPABASE_PUBLISH_MS  300000UL    // 5-min averaged aggregate

// ════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════════════════════════════════
#define I2C_SDA          8
#define I2C_SCL          9

#define ANEM_PIN          7
#define R_SHUNT         156.0f
#define WS_MAX_MPS       30.0f

#define WIND_DIR_PIN      4
#define WIND_DIR_SHUNT  150.0f

#define VEML6070_ADDR_L  0x38
#define VEML6070_ADDR_H  0x39

#define BMP390_ADDR      0x77
#define SEA_LEVEL_HPA  1008.0f

#define SHT85_ADDRESS    0x44

Adafruit_BMP3XX bmp;
SHT85 sht(SHT85_ADDRESS);
DFRobot_RainfallSensor_I2C rainSensor(&Wire);

// ════════════════════════════════════════════════════════════════════
//  MQTT / Networking globals
// ════════════════════════════════════════════════════════════════════
WiFiClientSecure mqttNet;
PubSubClient     mqttClient(mqttNet);

String topicData;
String topicStatus;
String mqttClientId;

unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_MS = 5000;

// ════════════════════════════════════════════════════════════════════
//  Wind Direction — 16-point compass lookup
// ════════════════════════════════════════════════════════════════════
const char* compassDir16[] = {
  "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
};

// ════════════════════════════════════════════════════════════════════
//  LATEST INSTANTANEOUS READINGS — used for the MQTT live snapshot
// ════════════════════════════════════════════════════════════════════
float windDir_mA       = 0;
float windDir_deg       = -1;
String windDir_compass  = "ERR";

float anem_mps = 0, anem_kmh = 0, anem_mA = 0;

float lastPressureHpa = 0;
float lastTempC       = 0;
float lastHumidityPct = 0;
uint16_t lastUvRaw     = 0xFFFF;

float latestRainCumulative = 0;   // mm, lifetime total from sensor

// ════════════════════════════════════════════════════════════════════
//  FAST-TIER ACCUMULATORS (wind, rain) — reset every Supabase publish
// ════════════════════════════════════════════════════════════════════
float windSpeedSum   = 0;
int   windSpeedCount = 0;
float windSpeedGust  = 0;         // max instantaneous speed this window

float windDirSumSin = 0;
float windDirSumCos = 0;
int   windDirCount   = 0;         // only valid (non-error) samples counted

float rainAtLastSupabasePublish = -1;  // -1 = not yet initialized

// ════════════════════════════════════════════════════════════════════
//  SLOW-TIER ACCUMULATORS (pressure, temp, humidity, UV)
// ════════════════════════════════════════════════════════════════════
float pressureSum = 0; int pressureCount = 0;
float tempSum     = 0; int tempCount     = 0;
float humiditySum = 0; int humidityCount = 0;
float uvSum       = 0; int uvCount       = 0;

float uvAvgPrevWindow = -1;   // previous 5-min UV avg, for sudden-drop flag

// ════════════════════════════════════════════════════════════════════
//  Scheduling
// ════════════════════════════════════════════════════════════════════
unsigned long lastFastSample     = 0;
unsigned long lastSlowSample     = 0;
unsigned long lastMqttPublish    = 0;
unsigned long lastSupabasePublish = 0;


// ════════════════════════════════════════════════════════════════════
//  Read Wind Direction (4-20mA current loop via 150Ω shunt on GPIO4)
// ════════════════════════════════════════════════════════════════════
void readWindDirection() {
  int   raw  = analogRead(WIND_DIR_PIN);
  float volt = (raw / 4095.0f) * 3.3f;
  float mA   = (volt / WIND_DIR_SHUNT) * 1000.0f;

  if (mA < 3.5f) {
    windDir_mA = mA;
    windDir_compass = "ERR:OpenLoop";
    windDir_deg = -1;
    return;
  }

  mA = constrain(mA, 4.0f, 20.0f);
  windDir_mA = mA;

  float x   = mA - 4.0f;
  int   idx = ((int)roundf(x)) % 16;
  if (idx < 0) idx += 16;

  windDir_deg     = idx * 22.5f;
  windDir_compass = String(compassDir16[idx]);
}

// ════════════════════════════════════════════════════════════════════
//  Read Anemometer (4–20 mA analog) — WIND SPEED
// ════════════════════════════════════════════════════════════════════
void readAnemometer() {
  int   raw  = analogRead(ANEM_PIN);
  float volt = (raw / 4095.0f) * 3.3f;
  anem_mA    = constrain((volt / R_SHUNT) * 1000.0f, 4.0f, 20.0f);
  anem_mps   = (anem_mA - 4.0f) / 16.0f * WS_MAX_MPS;
  anem_kmh   = anem_mps * 3.6f;
}

// ════════════════════════════════════════════════════════════════════
//  Read UV (VEML6070)
// ════════════════════════════════════════════════════════════════════
uint16_t readVEML6070() {
  uint16_t hi = 0, lo = 0;

  Wire.requestFrom((uint8_t)VEML6070_ADDR_H, (uint8_t)1);
  uint32_t t = millis();
  while (!Wire.available()) { if (millis()-t > 100) return 0xFFFF; }
  hi = Wire.read();

  Wire.requestFrom((uint8_t)VEML6070_ADDR_L, (uint8_t)1);
  t = millis();
  while (!Wire.available()) { if (millis()-t > 100) return 0xFFFF; }
  lo = Wire.read();

  return (hi << 8) | lo;
}

// ════════════════════════════════════════════════════════════════════
//  FAST TIER — wind + rain, sampled every FAST_SAMPLE_MS
// ════════════════════════════════════════════════════════════════════
void sampleFast() {
  readAnemometer();
  readWindDirection();

  // Wind speed: accumulate for avg, track gust (max)
  windSpeedSum += anem_mps;
  windSpeedCount++;
  if (anem_mps > windSpeedGust) windSpeedGust = anem_mps;

  // Wind direction: vector average (handles the 360°/0° wraparound
  // correctly, unlike a naive arithmetic mean of degrees)
  if (windDir_deg >= 0) {
    float rad = windDir_deg * (PI / 180.0f);
    windDirSumSin += sinf(rad);
    windDirSumCos += cosf(rad);
    windDirCount++;
  }

  // Rain: just refresh the latest cumulative reading; the interval
  // delta is computed at Supabase-publish time, not here.
  latestRainCumulative = rainSensor.getRainfall();
  if (rainAtLastSupabasePublish < 0) {
    // first-ever sample after boot — seed the baseline so the first
    // interval isn't inflated by the sensor's full lifetime total
    rainAtLastSupabasePublish = latestRainCumulative;
  }
}

// ════════════════════════════════════════════════════════════════════
//  SLOW TIER — pressure/temp/humidity/UV, sampled every SLOW_SAMPLE_MS
// ════════════════════════════════════════════════════════════════════
void sampleSlow() {
  if (bmp.performReading()) {
    lastPressureHpa = bmp.pressure / 100.0f;
    pressureSum += lastPressureHpa;
    pressureCount++;
  }

  sht.read();
  lastTempC = sht.getTemperature();
  lastHumidityPct = sht.getHumidity();
  tempSum += lastTempC;
  tempCount++;
  humiditySum += lastHumidityPct;
  humidityCount++;

  uint16_t uvRaw = readVEML6070();
  if (uvRaw != 0xFFFF) {
    lastUvRaw = uvRaw;
    uvSum += uvRaw;
    uvCount++;
  }
}

// ════════════════════════════════════════════════════════════════════
//  WiFi
// ════════════════════════════════════════════════════════════════════
void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(400);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" FAILED — will keep retrying in loop()");
  }
}

// ════════════════════════════════════════════════════════════════════
//  MQTT — non-blocking connect/reconnect with LWT
// ════════════════════════════════════════════════════════════════════
void mqttReconnect() {
  if (mqttClient.connected()) return;
  if (millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = millis();

  if (WiFi.status() != WL_CONNECTED) return;

  Serial.print("[MQTT] Connecting to HiveMQ... ");

  bool ok = mqttClient.connect(
    mqttClientId.c_str(),
    HIVEMQ_USERNAME, HIVEMQ_PASSWORD,
    topicStatus.c_str(), 1, true, "offline"
  );

  if (ok) {
    Serial.println("connected");
    mqttClient.publish(topicStatus.c_str(), "online", true);
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ════════════════════════════════════════════════════════════════════
//  MQTT PUBLISH — live snapshot for the app dashboard (every 30s)
//  Uses the LATEST instantaneous readings, not averages — this is
//  meant to feel "live", not smoothed.
// ════════════════════════════════════════════════════════════════════
void publishMqttSnapshot() {
  StaticJsonDocument<384> doc;
  doc["temp_c"]        = lastTempC;
  doc["humidity_pct"]  = lastHumidityPct;
  doc["pressure_hpa"]  = lastPressureHpa;
  doc["rain_mm"]       = latestRainCumulative;
  doc["wind_speed_ms"] = anem_mps;
  if (windDir_deg >= 0) doc["wind_dir_deg"] = windDir_deg;
  if (lastUvRaw != 0xFFFF) doc["uv_index"] = lastUvRaw;
  doc["ts"] = (uint32_t)(millis() / 1000);

  char payload[400];
  size_t n = serializeJson(doc, payload, sizeof(payload));

  if (mqttClient.connected()) {
    bool ok = mqttClient.publish(topicData.c_str(), (const uint8_t*)payload, n, true);
    Serial.print("[MQTT] live snapshot: ");
    Serial.println(ok ? "ok" : "FAILED");
  } else {
    Serial.println("[MQTT] skipped — not connected");
  }
}

// ════════════════════════════════════════════════════════════════════
//  SUPABASE PUBLISH — averaged aggregate for the trailing 5-min
//  window. This is the data the app's forecasting/trend logic reads.
// ════════════════════════════════════════════════════════════════════
void postAggregateToSupabase(float pressureAvg, float tempAvg, float humidityAvg,
                              float uvAvg, bool uvSuddenDrop,
                              float windSpeedAvg, float windSpeedGustVal,
                              float windDirAvg,
                              float rainIntervalMm, float rainCumulativeMm) {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();  // dev/test only — pin the CA for production

  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/rpc/log_reading";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);

  // NOTE: these param names must match your Supabase RPC signature.
  // The RPC needs updating from the old single-sample field set to
  // this aggregate field set before this will succeed.
  StaticJsonDocument<512> body;
  body["p_station_id"]          = STATION_ID;
  body["p_pressure_hpa_avg"]    = pressureAvg;
  body["p_temp_c_avg"]          = tempAvg;
  body["p_humidity_pct_avg"]    = humidityAvg;
  if (uvAvg >= 0) body["p_uv_raw_avg"] = uvAvg;
  body["p_uv_sudden_drop"]      = uvSuddenDrop;
  body["p_wind_speed_ms_avg"]   = windSpeedAvg;
  body["p_wind_speed_ms_gust"]  = windSpeedGustVal;
  if (windDirAvg >= 0) body["p_wind_dir_deg_avg"] = windDirAvg;
  body["p_rain_mm_interval"]    = rainIntervalMm;
  body["p_rain_mm_cumulative"]  = rainCumulativeMm;

  String payload;
  serializeJson(body, payload);

  int code = http.POST(payload);
  if (code > 0 && code < 300) {
    Serial.print("[Supabase] aggregate log OK, code ");
    Serial.println(code);
  } else {
    Serial.print("[Supabase] aggregate log FAILED, code ");
    Serial.println(code);
  }
  http.end();
}

// ════════════════════════════════════════════════════════════════════
//  Reduce the 5-min accumulators to an aggregate and publish, then
//  reset for the next window.
// ════════════════════════════════════════════════════════════════════
void publishAndResetAggregates() {
  float pressureAvg = (pressureCount > 0) ? (pressureSum / pressureCount) : lastPressureHpa;
  float tempAvg      = (tempCount > 0)     ? (tempSum / tempCount)         : lastTempC;
  float humidityAvg  = (humidityCount > 0) ? (humiditySum / humidityCount) : lastHumidityPct;
  float uvAvg         = (uvCount > 0) ? (uvSum / uvCount) : -1;

  bool uvSuddenDrop = false;
  if (uvAvg >= 0 && uvAvgPrevWindow >= 0) {
    // Flag if this window's average UV dropped more than 40% from
    // the previous window — a possible fast-moving-cloud/rain signal.
    uvSuddenDrop = (uvAvg < uvAvgPrevWindow * 0.6f);
  }
  if (uvAvg >= 0) uvAvgPrevWindow = uvAvg;

  float windSpeedAvg = (windSpeedCount > 0) ? (windSpeedSum / windSpeedCount) : 0;

  float windDirAvg = -1;
  if (windDirCount > 0) {
    float rad = atan2f(windDirSumSin, windDirSumCos);
    float deg = rad * (180.0f / PI);
    if (deg < 0) deg += 360.0f;
    windDirAvg = deg;
  }

  float rainInterval = 0;
  if (rainAtLastSupabasePublish >= 0) {
    rainInterval = latestRainCumulative - rainAtLastSupabasePublish;
    if (rainInterval < 0) rainInterval = 0;  // guard against a sensor reset
  }

  Serial.println("[Aggregate] Publishing 5-min window to Supabase:");
  Serial.print("  Pressure avg: "); Serial.print(pressureAvg, 2); Serial.println(" hPa");
  Serial.print("  Temp avg: ");     Serial.print(tempAvg, 2);     Serial.println(" C");
  Serial.print("  Humidity avg: "); Serial.print(humidityAvg, 2); Serial.println(" %RH");
  Serial.print("  UV avg: ");       Serial.print(uvAvg, 1);
  Serial.print("   Sudden drop: "); Serial.println(uvSuddenDrop ? "YES" : "no");
  Serial.print("  Wind avg: ");     Serial.print(windSpeedAvg, 2);
  Serial.print(" m/s   Gust: ");    Serial.print(windSpeedGust, 2); Serial.println(" m/s");
  Serial.print("  Wind dir avg: "); Serial.print(windDirAvg, 1); Serial.println(" deg");
  Serial.print("  Rain interval: ");Serial.print(rainInterval, 2);
  Serial.print(" mm   Cumulative: ");Serial.print(latestRainCumulative, 2); Serial.println(" mm");

  postAggregateToSupabase(pressureAvg, tempAvg, humidityAvg,
                           uvAvg, uvSuddenDrop,
                           windSpeedAvg, windSpeedGust,
                           windDirAvg,
                           rainInterval, latestRainCumulative);

  // Reset accumulators for the next 5-min window
  windSpeedSum = 0; windSpeedCount = 0; windSpeedGust = 0;
  windDirSumSin = 0; windDirSumCos = 0; windDirCount = 0;
  pressureSum = 0; pressureCount = 0;
  tempSum = 0; tempCount = 0;
  humiditySum = 0; humidityCount = 0;
  uvSum = 0; uvCount = 0;
  rainAtLastSupabasePublish = latestRainCumulative;
}

// ════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // VEML6070 init
  Wire.beginTransmission(VEML6070_ADDR_L);
  Wire.write(0x02);
  if (Wire.endTransmission() != 0)
    Serial.println("[WARN] VEML6070 init failed — check wiring");
  delay(200);

  // BMP390 init
  if (!bmp.begin_I2C(BMP390_ADDR)) {
    Serial.println("[WARN] BMP390 not found — check address/wiring");
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // SHT85 init
  sht.begin();
  delay(3000);

  // Tipping Bucket init (I2C)
  int retries = 3;
  while (!rainSensor.begin() && retries-- > 0) {
    Serial.println("[WARN] Rain sensor (I2C) init failed, retrying...");
    delay(1000);
  }
  if (retries <= 0) {
    Serial.println("[WARN] Rain sensor not detected on I2C — check address/wiring");
  }
  latestRainCumulative = rainSensor.getRainfall();
  rainAtLastSupabasePublish = latestRainCumulative;

  // Analog (ESP32-S3)
  analogReadResolution(12);
  analogSetPinAttenuation(ANEM_PIN, ADC_11db);
  analogSetPinAttenuation(WIND_DIR_PIN, ADC_11db);

  // Networking
  topicData    = String("weatherstation/") + STATION_ID + "/data";
  topicStatus  = String("weatherstation/") + STATION_ID + "/status";
  mqttClientId = String("esp32-") + STATION_ID;

  connectWiFi();

  mqttNet.setInsecure();  // dev/test only — pin the CA for production
  mqttClient.setServer(HIVEMQ_HOST, HIVEMQ_PORT);
  mqttClient.setBufferSize(512);
  mqttReconnect();

  unsigned long now = millis();
  lastFastSample = lastSlowSample = lastMqttPublish = lastSupabasePublish = now;

  Serial.println("════════════════════════════════════════════");
  Serial.println("  Unified Weather Station — Two-Tier Ready   ");
  Serial.println("════════════════════════════════════════════");
}

// ════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  mqttReconnect();
  mqttClient.loop();

  unsigned long now = millis();

  // FAST TIER — wind + rain
  if (now - lastFastSample >= FAST_SAMPLE_MS) {
    lastFastSample = now;
    sampleFast();
  }

  // SLOW TIER — pressure/temp/humidity/UV
  if (now - lastSlowSample >= SLOW_SAMPLE_MS) {
    lastSlowSample = now;
    sampleSlow();
  }

  // MQTT — live snapshot for the app
  if (now - lastMqttPublish >= MQTT_PUBLISH_MS) {
    lastMqttPublish = now;
    publishMqttSnapshot();
  }

  // SUPABASE — 5-min averaged aggregate for forecasting/trend data
  if (now - lastSupabasePublish >= SUPABASE_PUBLISH_MS) {
    lastSupabasePublish = now;
    publishAndResetAggregates();
  }
}
