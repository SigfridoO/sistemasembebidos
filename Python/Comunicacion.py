import serial

# --------- Opcional -------
# Para listar la lista de puertos disponibles 
import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports())
for port in ports:
    print(port)
# --------------------------

puertoSerie = serial.Serial()

puertoSerie.port =""
puertoSerie.baudrate = 9600
puertoSerie.parity = serial.PARITY_NONE
puertoSerie.timeout = 0
puertoSerie.stopbits = serial.STOPBITS_ONE
puertoSerie.bytesize = serial.EIGHTBITS