#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include "arm_math.h"
#include <stdint.h>

// Estrutura para armazenar resultados de potência
typedef struct {
  float active_power;    // Potência ativa (W)
  float reactive_power;  // Potência reativa (VAR)
  float apparent_power;  // Potência aparente (VA)
  float power_factor;    // Fator de potência
} Power_Results_t;

// Estrutura para armazenar resultados de qualidade
typedef struct {
  float rms_voltage;  // Tensão RMS (V)
  float rms_current;  // Corrente RMS (A)
  float thd_voltage;  // THD da tensão (%)
  float thd_current;  // THD da corrente (%)
} Quality_Results_t;

// Função para converter valores do ADC para Q15
q15_t convert_adc_to_q15(uint16_t adc_value);

// Função para converter valores Q15 para float
float convert_q15_to_float(q15_t q15_value);

// Funções de cálculo RMS
float32_t calculate_voltage_rms(q15_t *buffer, uint32_t length);
float32_t calculate_current_rms(q15_t *buffer, uint32_t length);

// Funções de cálculo de potência
Power_Results_t calculate_power(q15_t *voltage_buffer, q15_t *current_buffer, uint32_t length);

// Funções de cálculo de THD
float32_t calculate_voltage_thd(q15_t *buffer, uint32_t length);
float32_t calculate_current_thd(q15_t *buffer, uint32_t length);

// Função para calcular todos os parâmetros de qualidade
Quality_Results_t calculate_quality_parameters(q15_t *voltage_buffer, q15_t *current_buffer,
                                               uint32_t length);

#endif  // SIGNAL_PROCESSING_H