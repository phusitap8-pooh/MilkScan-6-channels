#include <Arduino.h>
#include <Wire.h>
#include <AS726X.h>

// ── Hardware config ───────────────────────────────────────────────────────────
#define TCA_ADDR       0x70
#define SENSOR_GAIN    2
#define SENSOR_MODE    3
#define SENSOR_INTEG   50
#define BULB_CURRENT   2
#define NUM_SENSORS    3
#define NUM_CHANNELS   6
#define MEAS_SAMPLES   20

static const char* CH_LABEL[NUM_CHANNELS] = {
    "V(450)","B(500)","G(550)","Y(570)","O(600)","R(650)"
};

// ── Stage 1 calibration: cal = gain*raw + bias ────────────────────────────────
static const float CAL_GAIN[NUM_SENSORS][NUM_CHANNELS] = {
    { 1.0578f, 1.1475f, 1.1545f, 1.1560f, 1.1837f, 1.0600f },
    { 1.0130f, 0.9987f, 0.9912f, 0.9663f, 0.8846f, 0.8471f },
    { 0.8477f, 0.8637f, 0.8801f, 0.9002f, 0.9305f, 1.0118f },
};
static const float CAL_BIAS[NUM_SENSORS][NUM_CHANNELS] = {
    { -409.90f, -808.15f, -1296.45f, -1620.99f, -3545.02f, -1546.75f },
    { 1143.58f,  877.92f,  1317.34f,  1710.92f,  3451.66f,  3228.05f },
    {  143.39f,   57.08f,   -69.23f,  -197.55f,   -71.18f,  -755.96f },
};

static AS726X sensor;

// ── I2C / TCA ─────────────────────────────────────────────────────────────────
static bool tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << ch);
    return Wire.endTransmission() == 0;
}
static void tcaDeselect() {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
}
static void waitEnter() {
    while (Serial.available()) Serial.read();
    while (!Serial.available());
    while (Serial.available()) Serial.read();
}
static bool initSensor(uint8_t ch) {
    if (!tcaSelect(ch)) { Serial.printf("[INIT] CH%d TCA FAIL\n", ch); return false; }
    if (!sensor.begin(Wire, SENSOR_GAIN, SENSOR_MODE)) {
        Serial.printf("[INIT] CH%d FAIL\n", ch); tcaDeselect(); return false;
    }
    sensor.setIntegrationTime(SENSOR_INTEG);
    sensor.setBulbCurrent(BULB_CURRENT);
    sensor.disableBulb();
    Serial.printf("[INIT] CH%d OK  HW=0x%02X\n", ch, sensor.getVersion());
    return true;
}

// ── Bulb helpers ──────────────────────────────────────────────────────────────
static void enableAllBulbs() {
    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        tcaSelect(s);
        sensor.enableBulb();
        tcaDeselect();
    }
}
static void disableAllBulbs() {
    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        tcaSelect(s);
        sensor.disableBulb();
        tcaDeselect();
    }
}

// ── Acquire → Stage1 → Stage2 ─────────────────────────────────────────────────
static bool acquireNorm(uint8_t s, float normOut[NUM_CHANNELS]) {
    long  sum[NUM_CHANNELS] = {};
    int   good = 0;
    tcaSelect(s);
    sensor.enableBulb();
    for (int i = 0; i < MEAS_SAMPLES; i++) {
        if (sensor.takeMeasurements() != 0) continue;
        int raw[NUM_CHANNELS] = {
            sensor.getViolet(), sensor.getBlue(),  sensor.getGreen(),
            sensor.getYellow(), sensor.getOrange(), sensor.getRed()
        };
        bool sat = false;
        for (int c = 0; c < NUM_CHANNELS; c++) if (raw[c] == 65535) { sat = true; break; }
        if (!sat) { for (int c = 0; c < NUM_CHANNELS; c++) sum[c] += raw[c]; good++; }
    }
    sensor.disableBulb();
    tcaDeselect();
    if (good == 0) return false;

    // Stage 1: gain + bias
    float cal[NUM_CHANNELS], total = 0;
    for (int c = 0; c < NUM_CHANNELS; c++) {
        cal[c] = CAL_GAIN[s][c] * ((float)sum[c] / good) + CAL_BIAS[s][c];
        if (cal[c] < 0) cal[c] = 0;
        total += cal[c];
    }
    // Stage 2: normalize
    for (int c = 0; c < NUM_CHANNELS; c++)
        normOut[c] = (total > 0) ? cal[c] / total : 0;
    return true;
}

// ── Real-time single-shot read (raw values, bulb on during read) ──────────────
static bool acquireRaw(uint8_t s, int rawOut[NUM_CHANNELS]) {
    if (!tcaSelect(s)) return false;
    sensor.enableBulb();
    bool ok = (sensor.takeMeasurements() == 0);
    if (ok) {
        rawOut[0] = sensor.getViolet();
        rawOut[1] = sensor.getBlue();
        rawOut[2] = sensor.getGreen();
        rawOut[3] = sensor.getYellow();
        rawOut[4] = sensor.getOrange();
        rawOut[5] = sensor.getRed();
    }
    sensor.disableBulb();
    tcaDeselect();
    return ok;
}

// ── Fast 1-sample calibrated+normalized read ──────────────────────────────────
static bool acquireNormFast(uint8_t s, float normOut[NUM_CHANNELS]) {
    int raw[NUM_CHANNELS] = {};
    if (!acquireRaw(s, raw)) return false;
    float cal[NUM_CHANNELS], total = 0;
    for (int c = 0; c < NUM_CHANNELS; c++) {
        cal[c] = CAL_GAIN[s][c] * raw[c] + CAL_BIAS[s][c];
        if (cal[c] < 0) cal[c] = 0;
        total += cal[c];
    }
    for (int c = 0; c < NUM_CHANNELS; c++)
        normOut[c] = (total > 0) ? cal[c] / total : 0;
    return true;
}

// ── Simultaneous: all bulbs stay on, read each sensor via TCA ────────────────
static bool acquireAllSimultaneous(float normOut[NUM_SENSORS][NUM_CHANNELS], bool okOut[NUM_SENSORS]) {
    int raw[NUM_SENSORS][NUM_CHANNELS] = {};
    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        okOut[s] = false;
        if (!tcaSelect(s)) { tcaDeselect(); continue; }
        if (sensor.takeMeasurements() == 0) {
            raw[s][0] = sensor.getViolet();
            raw[s][1] = sensor.getBlue();
            raw[s][2] = sensor.getGreen();
            raw[s][3] = sensor.getYellow();
            raw[s][4] = sensor.getOrange();
            raw[s][5] = sensor.getRed();
            okOut[s] = true;
        }
        tcaDeselect();
    }
    bool anyOk = false;
    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        if (!okOut[s]) continue;
        anyOk = true;
        float cal[NUM_CHANNELS], total = 0;
        for (int c = 0; c < NUM_CHANNELS; c++) {
            cal[c] = CAL_GAIN[s][c] * raw[s][c] + CAL_BIAS[s][c];
            if (cal[c] < 0) cal[c] = 0;
            total += cal[c];
        }
        for (int c = 0; c < NUM_CHANNELS; c++)
            normOut[s][c] = (total > 0) ? cal[c] / total : 0;
    }
    return anyOk;
}

// ── Measure all sensors, print normalized comparison ─────────────────────────
static void measureAll(const char* label) {
    float norm[NUM_SENSORS][NUM_CHANNELS] = {};
    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        if (!acquireNorm(s, norm[s]))
            Serial.printf("  S%d FAIL\n", s);
    }

    Serial.printf("\n[%s]\n", label);
    Serial.printf("  %-7s  %8s  %8s  %8s  %8s  %6s\n",
                  "Channel","S0","S1","S2","Mean","CV%");
    Serial.println("  -------------------------------------------------------");

    float totalCV = 0;
    float meanNorm[NUM_CHANNELS];
    for (int c = 0; c < NUM_CHANNELS; c++) {
        float mean = (norm[0][c]+norm[1][c]+norm[2][c]) / 3.0f;
        float var  = 0;
        for (int s = 0; s < NUM_SENSORS; s++) { float d = norm[s][c]-mean; var += d*d; }
        float cv = mean > 0 ? sqrtf(var/3.0f)/mean*100.0f : 0;
        totalCV += cv;
        meanNorm[c] = mean;
        Serial.printf("  %-7s  %8.4f  %8.4f  %8.4f  %8.4f  %5.2f%%  %s\n",
                      CH_LABEL[c],
                      norm[0][c], norm[1][c], norm[2][c], mean,
                      cv, cv<1.0f?"PASS":cv<2.0f?"OK":"CHECK");
    }
    Serial.printf("  Mean CV: %.2f%%\n", totalCV / NUM_CHANNELS);

    // Key ratios
    float b = meanNorm[1], o = meanNorm[4], r = meanNorm[5];
    Serial.printf("  B/O=%.4f  (O+R)/(V+B)=%.4f\n",
                  o>0?b/o:0,
                  (meanNorm[0]+b)>0?(o+r)/(meanNorm[0]+b):0);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println(" MilkScan — Real-Time Channel Monitor  ");
    Serial.println("========================================\n");

    Wire.begin(21, 22);
    Wire.beginTransmission(TCA_ADDR);
    if (Wire.endTransmission() != 0) { Serial.println("FATAL: TCA9548A not found."); while(1); }
    Serial.println("[INIT] TCA9548A OK");
    bool allOK = true;
    for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
        if (!initSensor(ch)) allOK = false;
        tcaDeselect();
    }
    if (!allOK) { Serial.println("FATAL: sensor init failed."); while(1); }
    Serial.println("[INIT] All sensors OK");
    enableAllBulbs();
    Serial.println("[INIT] All bulbs ON (stay on)");
    Serial.println("[INFO] Reading all channels in real-time...\n");
    Serial.printf("  %-4s  %-7s  %6s  %6s  %6s  %6s  %6s  %6s\n",
                  "Sens", "Ch", "V450", "B500", "G550", "Y570", "O600", "R650");
    Serial.println("  -------------------------------------------------------");
}

// ── Loop: all 3 bulbs on simultaneously, read each sensor via TCA ─────────────
void loop() {
    static uint32_t iter = 0;
    iter++;
    Serial.printf("--- #%lu ---\n", iter);
    Serial.printf("  %-4s  %8s  %8s  %8s  %8s  %8s  %8s\n",
                  "Sens", "V(450)", "B(500)", "G(550)", "Y(570)", "O(600)", "R(650)");

    float norm[NUM_SENSORS][NUM_CHANNELS] = {};
    bool  ok[NUM_SENSORS] = {};
    acquireAllSimultaneous(norm, ok);

    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        if (ok[s]) {
            Serial.printf("  S%-3d  %8.4f  %8.4f  %8.4f  %8.4f  %8.4f  %8.4f\n",
                          s, norm[s][0], norm[s][1], norm[s][2],
                             norm[s][3], norm[s][4], norm[s][5]);
        } else {
            Serial.printf("  S%-3d  FAIL\n", s);
        }
    }
    Serial.println();
}
