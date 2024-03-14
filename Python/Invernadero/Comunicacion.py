
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

    def enviar_mensaje():
        cadena = bytearray(1)
        r = None
        if self.puertoSerie.is_open:
            print('El puerto esta abierto')
            # while True:
            #     r = self.puertoSerie.read(1)
            #     print(r, int.from_bytes(r))
            numero = 2
            cadena[0]= numero

            # enviando al microntrontrolador
            self.puertoSerie.write(cadena)
            time.sleep(0.01)
            # leyendo del microcontrolador
            r = self.puertoSerie.read(8)
            print(r, int.from_bytes(r))
        
        return r    

    def probarPuerto(self):

        cadena = bytearray(1)
        r = None
        if self.puertoSerie.is_open:
            print('El puerto esta abierto')
            # while True:
            #     r = self.puertoSerie.read(1)
            #     print(r, int.from_bytes(r))
            numero = 2
            cadena[0]= numero

            # enviando al microntrontrolador
            self.puertoSerie.write(cadena)
            time.sleep(0.01)
            # leyendo del microcontrolador
            r = self.puertoSerie.read(8)
            print(r, int.from_bytes(r))
        
        return r 

def main():
    comu = Comunicacion("/dev/ttyUSB0", 9600)
    comu.abrirPuerto()
    comu.probarPuerto()
    comu.cerrarPuerto()

if __name__ == "__main__":
    main()