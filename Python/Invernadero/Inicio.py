import sys
from PyQt5.QtWidgets import QApplication
from Interfaz import Ventana

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Ventana()
    window.show();
    sys.exit(app.exec())
       