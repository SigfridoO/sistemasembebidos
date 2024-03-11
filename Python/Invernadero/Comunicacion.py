
import serial
import time

class Comunicacion:
    def __init__(self, nombre, baudrate):
        # --------------------------
        self.puertoSerie = serial.Serial()
        self.puertoSerie.port = nombre #"/dev/ttyUSB0"
        self.puertoSerie.baudrate = baudrate #9600
        self.puertoSerie.parity = serial.PARITY_NONE
        self.puertoSerie.timeout = 1
        self.puertoSerie.stopbits = serial.STOPBITS_ONE
        self.puertoSerie.bytesize = serial.EIGHTBITS

    def abrirPuerto(self):
        self.puertoSerie.open()

    def cerrarPuerto(self):
        self.puertoSerie.close()

    def leerPuerto(self):
        pass

    def escribir(self):
        pass

def main():
    comu = Comunicacion("/dev/ttyUSB0", 9600)

if __name__ == "__main__":
    main()