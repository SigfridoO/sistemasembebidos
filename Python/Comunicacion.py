import serial

# --------- Opcional -------
# Para listar la lista de puertos disponibles 
import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports())
for port in ports:
    print(port)
# --------------------------

puertoSerie = serial.Serial()

puertoSerie.port ="/dev/ttyUSB0"
puertoSerie.baudrate = 9600
puertoSerie.parity = serial.PARITY_NONE
puertoSerie.timeout = 1
puertoSerie.stopbits = serial.STOPBITS_ONE
puertoSerie.bytesize = serial.EIGHTBITS



puertoSerie.open()
if puertoSerie.is_open:
    print('El puerto esta abierto')
    while True:
        r = puertoSerie.read(1)
        print(r, int.from_bytes(r))
puertoSerie.close()
