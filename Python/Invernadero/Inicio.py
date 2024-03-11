import sys
from PyQt5.QtWidgets import QApplication
from Interfaz import Ventana
from Comunicacion import Comunicacion

class Inicio(Ventana):
    def __init__(self):
        Ventana.__init__(self)


        comu = Comunicacion("/dev/ttyUSB0", 9600)
        comu.abrirPuerto()
        comu.probarPuerto()
        comu.cerrarPuerto()




if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Inicio()
    window.show();
    sys.exit(app.exec())
       