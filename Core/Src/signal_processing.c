#include "signal_processing.h"

#define ADC_MAX 4095

q15_t convert_adc_to_q15(uint16_t adc_value) {
  // Primeiro normaliza para 0 a 1
  float normalized = (float)(adc_value - 2048) / (float)ADC_MAX;

  // Converte para Q15 (multiplica por 32768 e converte para q15_t)
  // 32768 = 2^15, que é o valor máximo positivo em Q15
  return (q15_t)(normalized * 32768.0f);
}

float convert_q15_to_float(q15_t q15_value) {
  // Converte Q15 para float dividindo por 32768 (2^15)
  return (float)q15_value / 32768.0f * ADC_MAX;
}

void apply_hanning_window(q15_t *buffer, uint32_t length) {
  float32_t window;
  float32_t windowed;

  for (uint32_t i = 0; i < length; i++) {
    // Calcula o coeficiente da janela Hanning
    window = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (length - 1)));

    // Aplica a janela ao sinal
    windowed = convert_q15_to_float(buffer[i]) * window;

    // Converte de volta para Q15
    buffer[i] = convert_adc_to_q15((uint16_t)(windowed + 2048));
  }
}

float32_t calculate_voltage_rms(q15_t *buffer, uint32_t length) {
  // Aplica janelamento antes do cálculo RMS
  apply_hanning_window(buffer, length);

  q15_t rms_value;
  arm_rms_q15(buffer, length, &rms_value);
  return convert_q15_to_float(rms_value);
}

float32_t calculate_current_rms(q15_t *buffer, uint32_t length) {
  q15_t rms_value;
  arm_rms_q15(buffer, length, &rms_value);
  return convert_q15_to_float(rms_value);
}

Power_Results_t calculate_power(q15_t *voltage_buffer, q15_t *current_buffer, uint32_t length) {
  Power_Results_t results = {0};
  // TODO: Implementar cálculos de potência
  return results;
}

float32_t calculate_voltage_thd(q15_t *buffer, uint32_t length) {
  // TODO: Implementar cálculo THD da tensão
  return 0.0f;
}

float32_t calculate_current_thd(q15_t *buffer, uint32_t length) {
  // TODO: Implementar cálculo THD da corrente
  return 0.0f;
}

Quality_Results_t calculate_quality_parameters(q15_t *voltage_buffer, q15_t *current_buffer,
                                               uint32_t length) {
  Quality_Results_t results = {0};
  // TODO: Implementar cálculos de qualidade
  return results;
}