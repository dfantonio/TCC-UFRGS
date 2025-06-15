import serial
import time
from datetime import datetime


def monitor_serial():
    # Configuração da porta serial
    ser = serial.Serial(
        port='COM5',
        baudrate=9600,
        timeout=1  # timeout de 1000ms (1 segundo)
    )

    print("Monitorando porta COM5...")
    print("Pressione Ctrl+C para sair")

    try:
        while True:
            if ser.in_waiting:  # Verifica se há dados disponíveis
                # Lê a linha da serial
                line = ser.readline().decode('utf-8').strip()

                # Obtém o timestamp atual
                timestamp = datetime.now().strftime(
                    "%Y-%m-%d %H:%M:%S.%f")[:-3]

                # Exibe a mensagem com timestamp
                print(f"[{timestamp}] {line}")

    except KeyboardInterrupt:
        print("\nMonitoramento encerrado pelo usuário")
    except serial.SerialException as e:
        print(f"Erro na comunicação serial: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("Porta serial fechada")


if __name__ == "__main__":
    monitor_serial()
