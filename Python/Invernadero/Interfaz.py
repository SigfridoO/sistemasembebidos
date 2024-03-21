
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, \
    QPushButton, QWidget,QLineEdit, QLabel ,\
    QVBoxLayout, QHBoxLayout, QGridLayout

from Indicador import Indicador

from Comunicacion import Comunicacion

class Ventana(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        boton = QPushButton("Escribir")
        self.texto = QLineEdit()
        self.etiquetaString = QLabel()
        etiquetaHex = QLabel()

        layoutVertical1 = QVBoxLayout()
        layoutHorizontal1 = QHBoxLayout()
        layoutHorizontal2 = QHBoxLayout()
        gridLayout = QGridLayout()

        layoutVertical1.addLayout(layoutHorizontal1)
        layoutVertical1.addLayout(layoutHorizontal2)
        layoutVertical1.addLayout(gridLayout)

        layoutHorizontal1.addWidget(self.texto)
        layoutHorizontal1.addWidget(boton)

        layoutHorizontal2.addWidget(self.etiquetaString)
        layoutHorizontal2.addWidget(etiquetaHex)

        indicadorX00 = Indicador("X_00", "Entrada 0")
        indicadorX01 = Indicador("X_01", "Entrada 1")
        indicadorX02 = Indicador("X_02", "Entrada 2")

        gridLayout.addWidget(indicadorX00, 1, 1)
        gridLayout.addWidget(indicadorX01, 2, 1)
        gridLayout.addWidget(indicadorX02, 3, 1)

        self.etiquetaString.setStyleSheet(f'background-color: white')
        etiquetaHex.setStyleSheet(f'background-color: white')
        widget = QWidget()
        widget.setLayout(layoutVertical1)

        

        self.setCentralWidget(widget)
        self.resize(400,300)
        boton.clicked.connect(self.realizar_accion)

    def realizar_accion(self):
        print("Botón clicado")

        comu = Comunicacion("/dev/ttyUSB0", 9600)
        comu.abrirPuerto()
        respuesta = comu.enviar_mensaje(int(self.texto.text()))
        self.etiquetaString.setText(respuesta.decode())
        print ('La respuesta recibida es ', respuesta)

        comu.cerrarPuerto()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Ventana()
    window.show();
    sys.exit(app.exec())
       
