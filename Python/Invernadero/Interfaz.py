
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, \
    QPushButton, QWidget,QLineEdit, QLabel ,\
    QVBoxLayout, QHBoxLayout

from Comunicacion import Comunicacion

class Ventana(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        boton = QPushButton("Escribir")
        texto = QLineEdit()
        etiquetaString = QLabel()
        etiquetaHex = QLabel()

        layoutVertical1 = QVBoxLayout()
        layoutHorizontal1 = QHBoxLayout()
        layoutHorizontal2 = QHBoxLayout()

        layoutVertical1.addLayout(layoutHorizontal1)
        layoutVertical1.addLayout(layoutHorizontal2)

        layoutHorizontal1.addWidget(texto)
        layoutHorizontal1.addWidget(boton)

        layoutHorizontal2.addWidget(etiquetaString)
        layoutHorizontal2.addWidget(etiquetaHex)

        etiquetaString.setStyleSheet(f'background-color: white')
        etiquetaHex.setStyleSheet(f'background-color: white')
        widget = QWidget()
        widget.setLayout(layoutVertical1)

        self.setCentralWidget(widget)
        boton.clicked.connect(self.realizar_accion)

    def realizar_accion(self):
        print("Botón clicado")

        comu = Comunicacion("/dev/ttyUSB0", 9600)
        comu.abrirPuerto()
        respuesta = comu.probarPuerto()
        print ('La respuesta recibida es ', respuesta)

        comu.cerrarPuerto()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Ventana()
    window.show();
    sys.exit(app.exec())
       
