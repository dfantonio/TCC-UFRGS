#ifndef ADC_TYPES_H
#define ADC_TYPES_H

#include <stdint.h>

typedef struct {
  uint16_t corrente;  // Canal 0 - Leitura de corrente
  uint16_t tensao;    // Canal 1 - Leitura de tensão
} ADC_Leituras_t;

#endif /* ADC_TYPES_H */