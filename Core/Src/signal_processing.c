#include "signal_processing.h"

#define ADC_MAX 4095
#define ADC_OFFSET 2048

float32_t convert_adc_to_float(uint16_t adc_value) {
  // Converte diretamente para float, removendo o offset
  return (float32_t)((adc_value) / (float32_t)ADC_MAX) * 3.3f;
}

void remove_offset(float32_t *buffer, uint32_t length) {
  float32_t sum = 0;
  for (uint32_t i = 0; i < length; i++) {
    sum += buffer[i];
  }
  float32_t offset = -sum / length;

  arm_offset_f32(buffer, offset, buffer, length);
}

void apply_hanning_window(float32_t *buffer, uint32_t length) {
  float32_t window;

  for (uint32_t i = 0; i < length; i++) {
    // Calcula o coeficiente da janela Hanning
    window = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (length - 1)));

    // Aplica a janela ao sinal
    buffer[i] *= window;
  }
}

float32_t calculate_voltage_rms(float32_t *buffer, uint32_t length) {
  // Aplica janelamento antes do cálculo RMS
  // apply_hanning_window(buffer, length);

  float32_t rms_value;
  arm_rms_f32(buffer, length, &rms_value);
  return rms_value;
}

float32_t calculate_current_rms(float32_t *buffer, uint32_t length) {
  float32_t rms_value;
  arm_rms_f32(buffer, length, &rms_value);
  return rms_value;
}

Power_Results_t calculate_power(float32_t *voltage_buffer, float32_t *current_buffer,
                                uint32_t length) {
  Power_Results_t results = {0};
  // TODO: Implementar cálculos de potência
  return results;
}

float32_t calculate_voltage_thd(float32_t *buffer, uint32_t length) {
  // TODO: Implementar cálculo THD da tensão
  return 0.0f;
}

float32_t calculate_current_thd(float32_t *buffer, uint32_t length) {
  // TODO: Implementar cálculo THD da corrente
  return 0.0f;
}

Quality_Results_t calculate_quality_parameters(float32_t *voltage_buffer, float32_t *current_buffer,
                                               uint32_t length) {
  Quality_Results_t results = {0};
  // TODO: Implementar cálculos de qualidade
  return results;
}