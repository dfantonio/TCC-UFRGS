#ifndef __SIGNAL_PROCESSING_H
#define __SIGNAL_PROCESSING_H

#include "arm_math.h"
#include <stdint.h>

// Estrutura para armazenar os resultados dos cálculos de potência
typedef struct {
  float32_t active_power;    // Potência ativa (W)
  float32_t reactive_power;  // Potência reativa (VAR)
  float32_t apparent_power;  // Potência aparente (VA)
  float32_t power_factor;    // Fator de potência
} Power_Results_t;

// Estrutura para armazenar os resultados dos cálculos de qualidade
typedef struct {
  float32_t rms_voltage;  // Tensão RMS (V)
  float32_t rms_current;  // Corrente RMS (A)
  float32_t thd_voltage;  // THD da tensão (%)
  float32_t thd_current;  // THD da corrente (%)
  float32_t frequency;    // Frequência (Hz)
} Quality_Results_t;

// Funções de conversão
float32_t convert_adc_to_float(uint16_t adc_value);

void remove_offset(float32_t *buffer, uint32_t length);

// Funções de processamento de sinal
void apply_hanning_window(float32_t *buffer, uint32_t length);

// Funções de cálculo
float32_t calculate_voltage_rms(float32_t *buffer, uint32_t length);
float32_t calculate_current_rms(float32_t *buffer, uint32_t length);
Power_Results_t calculate_power(float32_t *voltage_buffer, float32_t *current_buffer,
                                uint32_t length);
float32_t calculate_voltage_thd(float32_t *buffer, uint32_t length);
float32_t calculate_current_thd(float32_t *buffer, uint32_t length);
Quality_Results_t calculate_quality_parameters(float32_t *voltage_buffer, float32_t *current_buffer,
                                               uint32_t length);

/**
 * @brief Calcula o THD de um sinal
 * @param harmonics Ponteiro para o sinal de entrada (harmônicas)
 * @param thd_n É o array de saida que guarda a distorção harmônica de cada harmônica
 * @param length Tamanho do sinal
 * @return returns the THD of the signal
 */
float32_t calculate_thd(float32_t *harmonics, float32_t *thd_n, uint32_t length);

/**
 * @brief Calcula a FFT de um sinal
 * @param fft_instance Ponteiro para a instância da FFT
 * @param arr_in Ponteiro para o sinal de entrada
 * @param arr_temp Ponteiro para o sinal temporário
 * @param arr_out_mag Ponteiro para o sinal de saída
 * @param length Tamanho do sinal
 */
void calculate_fft(arm_rfft_fast_instance_f32 *fft_instance, float32_t *arr_in, float32_t *arr_temp,
                   float32_t *arr_out_mag, uint32_t length);

#endif /* __SIGNAL_PROCESSING_H */