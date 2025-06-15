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
  volatile static const float32_t scale_factor = 188.566;

  float32_t rms_value;
  arm_rms_f32(buffer, length, &rms_value);
  return rms_value * scale_factor;
}

float32_t calculate_current_rms(float32_t *buffer, uint32_t length) {
  volatile static const float32_t scale_factor = 85.7118;

  float32_t rms_value;
  arm_rms_f32(buffer, length, &rms_value);
  return rms_value * scale_factor;
}

Power_Results_t calculate_power(float32_t *voltage_buffer, float32_t *current_buffer,
                                uint32_t length) {
  Power_Results_t results = {0};
  // TODO: Implementar cálculos de potência
  return results;
}

void calculate_fft(arm_rfft_fast_instance_f32 *fft_instance, float32_t *arr_in, float32_t *arr_temp,
                   float32_t *arr_out_mag, uint32_t length) {
  apply_hanning_window(arr_in, length);

  arm_rfft_fast_f32(fft_instance, arr_in, arr_temp, 0);
  arm_cmplx_mag_f32(arr_temp, arr_out_mag, length);

  float32_t harmonics[50] = {0};

  // Calculates the amplitudes of the first 50 harmonics.
  for (uint32_t i = 1; i <= 50; i++) {
    int idx = 60 * i / 4.88;

    harmonics[i - 1] = arr_out_mag[idx - 2] + arr_out_mag[idx - 1] + arr_out_mag[idx] +
                       arr_out_mag[idx + 1] + arr_out_mag[idx + 2];
  }

  calculate_voltage_thd(harmonics, 50);
}

float32_t calculate_thd(float32_t harmonics[], float32_t thd_n[], uint32_t length) {
  float32_t fundamental = harmonics[0];
  float32_t sum_squares = 0.0f;

  // Calcula THD individual para cada harmônica
  for (uint32_t i = 1; i < length; i++) {
    thd_n[i] = harmonics[i] / fundamental;
    sum_squares += harmonics[i] * harmonics[i];
  }

  // Calcula THD total
  float32_t thd = sqrtf(sum_squares) / fundamental;

  return thd;
}

float32_t calculate_voltage_thd(float32_t *harmonics, uint32_t length) {
  float32_t thd_n[50] = {1};
  float32_t total_thd = calculate_thd(harmonics, thd_n, 50);

  return total_thd;
}

float32_t calculate_current_thd(float32_t *buffer, uint32_t length) {
  // TODO: Implementar cálculo THD da corrente
  return 0.0f;
}

Quality_Results_t calculate_quality_parameters(float32_t *voltage_buffer, float32_t *current_buffer,
                                               uint32_t length) {
  Quality_Results_t results = {0};

  // Calcula RMS
  results.rms_voltage = calculate_voltage_rms(voltage_buffer, length);
  results.rms_current = calculate_current_rms(current_buffer, length);

  // Calcula frequência (assumindo frequência de amostragem de 10kHz)
  const float32_t sampling_frequency = 10000.0f;
  results.frequency = calculate_frequency(voltage_buffer, length, sampling_frequency);

  // calculate_fft(&fft_instance, voltage_buffer, fft_temp, fft_mag, FFT_LENGTH / 2);
  // TODO: Implementar cálculos de THD
  results.thd_voltage = 0.0f;
  results.thd_current = 0.0f;

  return results;
}

float32_t calculate_frequency(float32_t *buffer, uint32_t length, float32_t sampling_frequency) {
  uint32_t zero_crossings = 0;
  float32_t total_period = 0.0f;
  float32_t prev_sample = buffer[0];
  float32_t prev_index = 0.0f;

  // Conta os cruzamentos por zero com interpolação linear
  for (uint32_t i = 1; i < length; i++) {
    float32_t current_sample = buffer[i];

    // Detecta cruzamento por zero (mudança de sinal)
    if ((prev_sample < 0 && current_sample >= 0) || (prev_sample > 0 && current_sample <= 0)) {
      // Calcula o ponto exato do cruzamento por zero usando interpolação linear
      float32_t crossing_point =
          (float32_t)(i - 1) + (-prev_sample) / (current_sample - prev_sample);

      if (zero_crossings > 0) {
        // Calcula o período entre este cruzamento e o anterior
        float32_t period = crossing_point - prev_index;
        total_period += period;
      }

      prev_index = crossing_point;
      zero_crossings++;
    }

    prev_sample = current_sample;
  }

  // Calcula a frequência média
  // Se não houver cruzamentos suficientes, retorna 0
  if (zero_crossings < 2) {
    return 0.0f;
  }

  float32_t avg_period = total_period / (zero_crossings - 1);
  float32_t frequency = sampling_frequency / (avg_period * 2);

  return frequency;
}