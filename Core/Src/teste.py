import math
import numpy as np
import matplotlib.pyplot as plt
from mock_data import MOCK_ADC_DATA


def calcular_rms(sinal):
    """
    Calcula o valor RMS (Root Mean Square) de um sinal.

    Args:
        sinal (list): Lista contendo as amostras do sinal

    Returns:
        float: Valor RMS do sinal
    """
    # Calcula a soma dos quadrados
    soma_quadrados = sum(x * x for x in sinal)

    # Calcula a média dos quadrados
    media_quadrados = soma_quadrados / len(sinal)

    # Calcula a raiz quadrada da média
    rms = math.sqrt(media_quadrados)

    return rms


def remove_offset(sinal):
    """
    Remove o offset de um sinal calculando a média antes de subtrair.

    Args:
        sinal (list): Lista contendo as amostras do sinal

    Returns:
        list: Sinal com offset removido
    """
    # Calcula a média do sinal
    media = sum(sinal) / len(sinal)

    # Remove o offset subtraindo a média
    return [x - media for x in sinal]


def converter_para_float(sinal):
    """
    Converte um sinal para float.
    """
    return [x / 4095 * 3.3 for x in sinal]


def calcular_fft(sinal, fs=10000):
    """
    Calcula a FFT do sinal usando numpy.

    Args:
        sinal: Array com as amostras do sinal
        fs: Frequência de amostragem (default 10kHz)

    Returns:
        freqs: Array com as frequências
        magnitudes: Array com as magnitudes da FFT
    """
    # Aplica janela de Hanning
    window = np.hanning(len(sinal))
    sinal_windowed = sinal * window

    # Calcula FFT
    fft_result = np.fft.rfft(sinal_windowed)

    # Calcula magnitudes
    magnitudes = np.abs(fft_result)

    # Calcula frequências
    freqs = np.fft.rfftfreq(len(sinal), 1/fs)

    return freqs, magnitudes


def calcular_thd(harmonicas):
    """
    Calcula o THD (Total Harmonic Distortion) de um sinal.

    Args:
        harmonicas: Array com as amplitudes das harmônicas

    Returns:
        tuple: (THD total, THD individual de cada harmônica)
    """
    fundamental = harmonicas[0]
    thd_n = np.zeros(len(harmonicas))
    soma_quadrados = 0.0

    # Calcula THD individual para cada harmônica
    for i in range(1, len(harmonicas)):
        thd_n[i] = harmonicas[i] / fundamental
        soma_quadrados += harmonicas[i] * harmonicas[i]

    # Calcula THD total
    thd_total = math.sqrt(soma_quadrados) / fundamental

    return thd_total, thd_n


def calcular_harmonicas(magnitudes, offset_basico=12):
    """
    Calcula as amplitudes das primeiras 50 harmônicas.

    Args:
        magnitudes: Array com as magnitudes da FFT
        offset_basico: Offset para encontrar as harmônicas (default 12)

    Returns:
        array: Amplitudes das harmônicas
    """
    harmonicas = np.zeros(50)
    for i in range(1, 51):
        idx = offset_basico * i
        harmonicas[i-1] = np.sum(magnitudes[idx-2:idx+3])
    return harmonicas


# Teste das funções
valor_float = converter_para_float(MOCK_ADC_DATA)
valor_float_offset = remove_offset(valor_float)

# Completa o array com zeros até o tamanho do array original
valor_float_offset = np.pad(
    valor_float_offset, (0, 48), 'constant', constant_values=0)

# Calcula FFT
freqs, magnitudes = calcular_fft(valor_float_offset)

# Calcula harmônicas
harmonicas = calcular_harmonicas(magnitudes)

# Calcula THD
thd_total, thd_n = calcular_thd(harmonicas)

print(f"Tamanho do sinal: {len(valor_float_offset)}")

# Exibe resultados
print("\nResultados do THD:")
print(f"THD Total: {thd_total*100:.2f}%")
print("\nTHD Individual por Harmônica:")
for i in range(1, 10):
    print(f"Harmônica {i}: {thd_n[i]*100:.2f}%")

# Plota o espectro de frequência
plt.figure(figsize=(12, 6))
plt.plot(freqs, magnitudes)
plt.title('Espectro de Frequência')
plt.xlabel('Frequência (Hz)')
plt.ylabel('Magnitude')
plt.grid(True)
plt.xlim(0, 400)  # Define o limite máximo do eixo x em 300 Hz
plt.show()
