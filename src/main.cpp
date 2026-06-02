#include <Arduino.h>
#include <esp_adc_cal.h>

// ADC1 channels only — avoids WiFi/BT conflict on ADC2
// GPIO34=ADC1_CH6, 35=ADC1_CH7, 32=ADC1_CH4, 33=ADC1_CH5, 36=ADC1_CH0, 39=ADC1_CH3
static const uint8_t CHANNEL_PINS[6] = {34, 35, 32, 33, 36, 39};
static const uint8_t NUM_CHANNELS = 6;

static uint16_t readings[NUM_CHANNELS];
static esp_adc_cal_characteristics_t adc_chars;

void setup() {
    Serial.begin(115200);
    Serial.println("[MilkScan] Init 6-channel ADC");
    Serial.println("Hello pooh");

    // 12-bit resolution, 0–3.3V range (11dB attenuation)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Calibrate ADC1
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
}

void loop() {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        uint32_t raw = analogRead(CHANNEL_PINS[i]);
        // Convert raw to millivolts using calibration
        readings[i] = (uint16_t)esp_adc_cal_raw_to_voltage(raw, &adc_chars);
    }

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        Serial.printf("CH%d: %4u mV  ", i, readings[i]);
    }
    Serial.println();

    delay(500);
}
