import math

valor = [2048,
         2305,
         2557,
         2802,
         3034,
         3251,
         3450,
         3626,
         3777,
         3901,
         3995,
         4059,
         4091,
         4091,
         4059,
         3995,
         3901,
         3777,
         3626,
         3450,
         3251,
         3034,
         2802,
         2557,
         2305,
         2048,
         1791,
         1539,
         1294,
         1062,
         845,
         646,
         470,
         319,
         195,
         101,
         37,
         5,
         5,
         37,
         101,
         195,
         319,
         470,
         646,
         845,
         1062,
         1294,
         1539,
         1791,
         2048,
         2305,
         2557,
         2802,
         3034,
         3251,
         3450,
         3626,
         3777,
         3901,
         3995,
         4059,
         4091,
         4091,
         4059,
         3995,
         3901,
         3777,
         3626,
         3450,
         3251,
         3034,
         2802,
         2557,
         2305,
         2048,
         1791,
         1539,
         1294,
         1062,
         845,
         646,
         470,
         319,
         195,
         101,
         37,
         5,
         5,
         37,
         101,
         195,
         319,
         470,
         646,
         845,
         1062,
         1294,
         1539,
         1791,
         2048,
         ]


def calcular_rms(sinal):
    """
    Calcula o valor RMS (Root Mean Square) de um sinal.

    Args:
        sinal (list): Lista contendo as amostras do sinal

    Returns:
        float: Valor RMS do sinal
    """
    # Remove o offset do sinal
    sinal = [x - 2048 for x in sinal]

    # Calcula a soma dos quadrados
    soma_quadrados = sum(x * x for x in sinal)

    # Calcula a média dos quadrados
    media_quadrados = soma_quadrados / len(sinal)

    # Calcula a raiz quadrada da média
    rms = math.sqrt(media_quadrados)

    return rms


print(len(valor))

# Exemplo de uso
rms_valor = calcular_rms(valor)
print(f"Valor RMS do sinal: {rms_valor:.2f}")
