// ----------  EEPROM
#include <EEPROM.h>

// ----------  Señales Digitales
#define numeroDeX 16
byte X[numeroDeX];


int DI_00 = 36;
int DI_01 = 39;
int DI_02 = 34;
int DI_03 = 35;

#define numeroDeY 16
byte Y[numeroDeY];

int DO_00 = 26;
int DO_01 = 27;
int DO_02 = 14;
int DO_03 = 12;
int DO_04 = 13;
int DO_05 = 2;

void leerPines (void);

#define numeroDeM 80
byte M[numeroDeM];

#define numeroDeRC 80
byte RC[numeroDeRC];

// ----------  Señales Analogicas
void leerAnalog(void);


// ----------  Temporizadores

//const int numeroDeTON = 16;
#define  numeroDeTON 16
struct temporizador {
  byte entrada;
  byte salida;
  unsigned long tiempo;
  unsigned long tiempoActual;
} TON[numeroDeTON];
struct temporizadorAux {
  byte bandera;
  unsigned long tiempo_Aux1;
  unsigned long tiempo_Aux2;
} TON_Aux[numeroDeTON];

void actualizarTON (int);
void configurarTemporizador(void);
// ----------  Contadores

#define numeroDeContadores 8
struct contador {
  byte entrada;
  byte salida;
  byte habilitar;
  int cuentaActual;
  int cuentaMaxima;
  byte reset;
  byte aux1;
  byte aux2;
} C[numeroDeContadores];

void actualizarContador (byte);


// ----------  Comunicación


/*-----------------------------------------------------------------*/
/*
     00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20|
    |CI|  TP | NUM | LON |  |  |  |  |  |  | ............. |NV|CF  |
    |  |     |     |     |  |  |

    CI   - 1 Byte   - Caracter de inicio
    TI   - 2 Byte   - Tipo de instrucción
    NUM  - 2 Byte   - Número de instrucción
    LON  - 2 Byte   - Longitud de la trama
    NV   - 1 Byte   - Número de verificación
    PARA - Variable - Parámetros
    CF   - 1 Byte   - Caracter final
*/


#define bufferTextoMaximo 120
byte bufferTexto [bufferTextoMaximo];
int bufferTextoIndice = 0;

#define bufferIndiceMaximo 100
byte bufferLectura [bufferIndiceMaximo];
int bufferIndice = 0;
byte bufferUltimaInstruccionByte [bufferIndiceMaximo];
int tamanioBufferUltimaInstruccion = 0;

void colocarEnBufferDatosSerial(void);
void imprimirTrama (byte *, int , int );
void leerInstruccionesDeBufferSerial(byte* , int* , byte*, int*, int);
int obtenerListaDeInstrucciones (void);

char caracterDeInicio = '-';
int textoTipoInstruccion = 0;
int textoInstruccion  = 0;
byte textoMascara = 0;
int textoLongitud = 0;
char caracterDeFin = '*';



// Tipo de instrucción
const byte ADMINISTRACION = 1;
const byte PROCESO = 2;

// Instrucciones para ADMINISTRACION
#define VERSION 180

// Instrucciones para PROCESO

#define ENTRADAS  12
#define ENVIAR_STATUS_SENALES 24
#define CONFIGURAR_SENALES_DIGITALES 25
#define TEMPERATURA 56

#define INSTRUCCION_MOSTRAR_VERSION 58

#define INDICE_DATOS 15
// ----------  Funciones utilitarias

byte obtenerByteDeArregloByte (byte*);
int obtenerIntDeArregloByte (byte*);
long obtenerLongDeArregloByte (byte*);
float obtenerFloatDeArregloByte (byte*);

void imprimirInicioTexto(int, int);
void imprimirFinalTexto();

void escribirByteEnTexto (byte);
void escribirNumeroIntEnTexto (int);
void escribirNumeroLongEnTexto (long);
void escribirNumeroFloatEnTexto (float);
void escribirCaracterEnTexto(char);
void escribirStringEnTexto (String);
void escribirStringEnTexto (char*);


// ----------  Variables

float analog_temperatura = 0;


// ----------  EEPROM
#define tamanioMemoriaEEPROM 512


void setup() {

  pinMode(DI_00, INPUT);
  pinMode(DI_01, INPUT);
  pinMode(DI_02, INPUT);
  pinMode(DI_03, INPUT);
  

  pinMode(DO_00, OUTPUT);
  pinMode(DO_01, OUTPUT);
  pinMode(DO_02, OUTPUT);
  pinMode(DO_03, OUTPUT);
  pinMode(DO_04, OUTPUT);
  pinMode(DO_05, OUTPUT);


  configurarTemporizador();

  EEPROM.begin(tamanioMemoriaEEPROM);

  Serial.begin(9600);

  Serial1.begin (9600);
  Serial1.setTimeout(3);


}


void loop() {

  leerPines();
  leerAnalog();
  colocarEnBufferDatosSerial ();
  leerInstruccionesDeBufferSerial(bufferLectura, &bufferIndice, bufferUltimaInstruccionByte, &tamanioBufferUltimaInstruccion);


  Y[0] = X[0];
  Y[1] = X[1];
  Y[2] = X[2];
  Y[3] = X[3];

  TON[0].entrada = !TON[1].salida;
  actualizarTON(0);

  TON[1].entrada = TON[0].salida;
  actualizarTON(1);

  Y[4] = !TON[1].salida & TON[0].salida; 





  TON[2].entrada = !TON[3].salida;
  actualizarTON(2);

  TON[3].entrada = TON[2].salida;
  actualizarTON(3);

  Y[5] = !TON[3].salida & !TON[2].salida; 

  
//  Serial.print ("\n");
//  Serial.print (TON[0].salida);
//  Serial.print ("\t");
//  Serial.print (TON[0].tiempoActual);
//    
//  Serial.print ("\t");
//  Serial.print (Y[4]);
//  


  
}



void leerPines () {
  X[0] = digitalRead(DI_00);
  X[1] = digitalRead(DI_01);
  X[2] = digitalRead(DI_02);
  X[3] = digitalRead(DI_03);

  digitalWrite(DO_00, Y[0]);
  digitalWrite(DO_01, Y[1]);
  digitalWrite(DO_02, Y[2]);
  digitalWrite(DO_03, Y[3]);
  digitalWrite(DO_04, Y[4]);
  digitalWrite(DO_05, Y[5]);
}

void leerAnalog () {

}




void actualizarTON (int i) {
  if (TON [i].entrada)
  {
    if (!TON_Aux[i].bandera) {
      TON_Aux[i].bandera = true;
      TON_Aux[i].tiempo_Aux1 = millis ();
    }
    TON_Aux[i].tiempo_Aux2 = millis ();
    TON [i].tiempoActual = TON_Aux[i].tiempo_Aux2 - TON_Aux[i].tiempo_Aux1;

    if (TON [i].tiempoActual > TON [i].tiempo) {
      TON [i].salida = true;
    }
  } else {
    TON [i].salida = false;
    TON_Aux[i].bandera = false;
  }
}

void configurarTemporizador () {

  TON[0].tiempo = (unsigned long)1 * 1000;
  TON[1].tiempo = (unsigned long)2 * 1500;

  TON[2].tiempo = (unsigned long)1 * 800;
  TON[3].tiempo = (unsigned long)1 * 400;
  TON[4].tiempo = (unsigned long)1 * 80;
  }




void actualizarContador (byte numeroContador) {

  if (!C[numeroContador].habilitar) {
    C[numeroContador].salida = 0;
    C[numeroContador].cuentaActual = 0;
  } else {
    C[numeroContador].aux2 = C[numeroContador].aux1 & !C[numeroContador].entrada;

    if (C[numeroContador].aux2) {
      if (C[numeroContador].cuentaActual < C[numeroContador].cuentaMaxima) {
        C[numeroContador].cuentaActual++;
      }

    }
  }

  if (C[numeroContador].reset) {
    C[numeroContador].cuentaActual = 0;
  }
  if (C[numeroContador].cuentaActual >= C[numeroContador].cuentaMaxima ) {
    C[numeroContador].salida = 1;
  } else {
    C[numeroContador].salida = 0;
  }

  C[numeroContador].aux1 = C[numeroContador].entrada;
  C[numeroContador].reset = 0;
}

void configurarContador () {
  C[0].cuentaMaxima = 3;
  C[1].cuentaMaxima = 6;
  C[2].cuentaMaxima = 6;
  C[3].cuentaMaxima = 1;
  C[4].cuentaMaxima = 1;
}


// ----------  Comunicación
void colocarEnBufferDatosSerial(void) {
  byte caracter;
  int aux = 0;
  int indiceAuxiliar = 0;

  while (Serial.available()) {
    caracter = Serial.read();
    bufferLectura [bufferIndice++] = caracter;

    //imprimirTrama(bufferLectura, 0, bufferIndice);

    if (bufferIndice > bufferIndiceMaximo - 25) {
      aux = bufferIndiceMaximo >> 1;
      for (int i = aux; i < bufferIndiceMaximo; i++) {
        bufferIndice = i - aux;
        bufferLectura[bufferIndice] = bufferLectura[i];
      }
    }
  }
}

void leerInstruccionesDeBufferSerial(byte* ptrBufferLectura, int* ptrBufferIndice,
                                     byte* ptrBufferUltimaInstruccionByte, int* ptrTamanioBufferUltimaInstruccion) {
  int encontrado = -1;
  int resultado = 0;


  int i = 0;
  int k = 0;

  if (ptrBufferLectura [*ptrBufferIndice - 1] ==  caracterDeFin ) {
    //Serial.print ("Se encontro el caracter de fin en ");
    //Serial.print (*ptrBufferIndice - 1 );
    //imprimirTrama(ptrBufferLectura, 0, *ptrBufferIndice);

    encontrado = -1;

    for (k = *ptrBufferIndice; k >= 0; --k) {

      //Serial.print(k);
      if (ptrBufferLectura[k] == (byte)caracterDeInicio) {

        //Serial.print ("Se encontro el caracter de inicio en ");
        //Serial.print (k);
        encontrado = k;

        if (encontrado >= 0) {

          //Serial.println ("ENCOTRADO ES MAYOR A CERO");
          *ptrTamanioBufferUltimaInstruccion = 0;

          for (int j = k;  j < *ptrBufferIndice ; j++) {
            ptrBufferUltimaInstruccionByte[*ptrTamanioBufferUltimaInstruccion] = ptrBufferLectura[j];
            (*ptrTamanioBufferUltimaInstruccion)++;
          }

          //Serial.print ("Imprimiendo trama");
          //imprimirTrama(ptrBufferUltimaInstruccionByte, 0, *ptrTamanioBufferUltimaInstruccion);
          int resultado = 0;

          if (verificarTrama ()) {
            //Serial.print ("VERIFICACION CORRECTA\n");

            resultado = obtenerListaDeInstrucciones ();

            *ptrBufferIndice = k;
          }
        }
        
      }
    }

    //Serial.print ("EXTRAIDO DE BUFFERLECTURA \n ENCONTRADO ES IGUAL A ");
    //Serial.println (encontrado);
    //imprimirTrama(ptrBufferLectura, k, *ptrBufferIndice-k);

  } 
}



int verificarTrama () {
  int* tamanio = 0;
  byte* cadena;
  int salida = 0;

  int reservado01 = 0;
  int reservado02 = 0;
  long numeroConsecutivo = 0;
  int tipoDeInstruccion = 0;
  int numeroDeInstruccion = 0;
  int longitudDeLaTrama = 0;
  byte verificacion = 0;

  tamanio = &tamanioBufferUltimaInstruccion;
  cadena = bufferUltimaInstruccionByte;
  int verif =  0;


  reservado01  = obtenerIntDeArregloByte (cadena + 1);
  reservado02  = obtenerIntDeArregloByte (cadena + 3);
  numeroConsecutivo  = obtenerLongDeArregloByte (cadena + 5);
  tipoDeInstruccion = obtenerIntDeArregloByte (cadena + 9);
  numeroDeInstruccion = obtenerIntDeArregloByte (cadena + 11);
  longitudDeLaTrama = obtenerIntDeArregloByte (cadena + 13);
  verificacion = obtenerByteDeArregloByte (cadena + *tamanio - 2);


  verif = 0;

  for (int i = 0; i < *tamanio - 2 ; i++) {
    verif ^= (byte) cadena[i];
    //Serial.print(">>");
    //Serial.print (verif, HEX);
    //Serial.println("<<");

  }

  /*

    Serial.print ("Rerservado 1 >>");
    Serial.print (reservado01);
    Serial.println("<<");

    Serial.print ("Rerservado 2 >>");
    Serial.print (reservado02);
    Serial.println("<<");

    Serial.print ("NumeroConsecutivo >>");
    Serial.print (numeroConsecutivo);
    Serial.println("<<");

    Serial.print ("tipoDeInstruccion >>");
    Serial.print (tipoDeInstruccion);
    Serial.println("<<");

    Serial.print ("numeroDeInstruccion >>");
    Serial.print (numeroDeInstruccion);
    Serial.println("<<");

    Serial.print ("longitudDeLaTrama >>");
    Serial.print (longitudDeLaTrama);
    Serial.println("<<");

    Serial.print ("verificacion >>");
    Serial.print (verificacion);
    Serial.println("<<");


    Serial.print ("tamanioCadena >>");
    Serial.print (*tamanio);
    Serial.println("<<");

    Serial.print ("verif >>");
    Serial.print (verif);
    Serial.println("<<");

  */

  if (longitudDeLaTrama == *tamanio) {
    //Serial.println ("Tamanio Correcto");
    if (verificacion == verif) {
      //Serial.println ("Verficacion correcta");
      return (true);
    }
  }

  return false;
}



int obtenerListaDeInstrucciones () {
  int* tamanio = 0;
  byte* cadena;
  int salida = 0;
  int tipoDeInstruccion = 0;
  int numeroDeInstruccion = 0;

  tamanio = &tamanioBufferUltimaInstruccion;
  cadena = bufferUltimaInstruccionByte;

  tipoDeInstruccion = obtenerIntDeArregloByte (cadena + 9);
  numeroDeInstruccion = obtenerIntDeArregloByte (cadena + 11);
  

  if (tipoDeInstruccion == ADMINISTRACION) {
    switch (numeroDeInstruccion) {
      case 0:
        break;

      case VERSION:
        imprimirVersion();
        break;
    }
  }

  if (tipoDeInstruccion == PROCESO) {
    switch (numeroDeInstruccion) {
      case CONFIGURAR_SENALES_DIGITALES:
        configurarSenalesDigitales();
        break;
      case ENVIAR_STATUS_SENALES:
        enviarStatusSenales();
        break;
      case TEMPERATURA:
        enviarTemperatura();
        break;
    }
  }
}
// ----------  Funciones
void imprimirVersion () {
  imprimirInicioTexto(ADMINISTRACION, INSTRUCCION_MOSTRAR_VERSION);

  escribirStringEnTexto(F("Version "));
  escribirStringEnTexto(F("0.1 "));
  escribirStringEnTexto(F("ESP32 "));
  imprimirFinalTexto();
}


void configurarSenalesDigitales () {
  int* tamanio = 0;
  byte* cadena;
  int numero = 0;
  int numeroAux = 0;
  byte valorCorrecto = 0;

  //Serial.println ("Dentro de senales digitales");
  tamanio = &tamanioBufferUltimaInstruccion;
  cadena = bufferUltimaInstruccionByte;
  int j = 0;
  for (int i = INDICE_DATOS; i < *tamanio - 2; i++) {
    j = i - INDICE_DATOS;
    numeroAux = obtenerByteDeArregloByte (cadena + i);
    //Serial.println (numeroAux);

    if (j % 2 == 0) { // par
      numero = numeroAux;
    }

    if (j % 2 == 1) { // impar

      if (numeroAux == 1) {
        if (numero < 6) {
          M[numero] = numeroAux;

          /*
            Serial.print ("M[");
            Serial.print (numero);
            Serial.print ("] = ");
            Serial.print (numeroAux);
          */
        }
      }
      if (numeroAux == 0) {
        if (numero < 6) {
          M[numero] = numeroAux;
          /*
            Serial.print ("M[");
            Serial.print (numero);
            Serial.print ("] = ");
            Serial.print (numeroAux);
          */
        }
      }
    }
  }
}

void enviarStatusSenales () {
  byte aux = 0;
  byte num_bytes = 0;

  bufferTextoIndice = 0;
  bufferTexto[bufferTextoIndice++] = '-';

  num_bytes = 2;
  for (int j = 0; j < num_bytes; j++) {
    aux = 0;
    for (int i = 0; i < 8; i++)
      aux |= X[i + j * 8] << i;
    bufferTexto[bufferTextoIndice++] = aux;
  }

  num_bytes = 2;
  for (int j = 0; j < num_bytes; j++) {
    aux = 0;
    for (int i = 0; i < 8; i++)
      aux |= Y[i + j * 8] << i;
    bufferTexto[bufferTextoIndice++] = aux;
  }

  bufferTexto[bufferTextoIndice++] = '*';
  Serial.write(bufferTexto, bufferTextoIndice);
}



void enviarTemperatura () {
  float numeroFloat = analog_temperatura;

  imprimirInicioTexto(PROCESO, TEMPERATURA);
  escribirNumeroFloatEnTexto (analog_temperatura);
  imprimirFinalTexto();
}

// ----------  Funciones utilitarias



byte obtenerByteDeArregloByte (byte* arreglo) {
  byte *punteroByte;
  punteroByte = (byte*) arreglo;
  return *punteroByte;
}


int obtenerIntDeArregloByte (byte* arreglo) {
  int *punteroInt;
  punteroInt = (int*) arreglo;
  return *punteroInt;
}

long obtenerLongDeArregloByte (byte* arreglo) {
  long *punteroLong;
  punteroLong = (long*) arreglo;
  return *punteroLong;
}

float obtenerFloatDeArregloByte (byte* arreglo) {
  float *punteroFloat;
  punteroFloat = (float*) arreglo;
  return *punteroFloat;
}


void bufferTextoLimpiar (void) {
  for (int i = 0; i < bufferTextoMaximo; i++) {
    bufferTexto[i] = (byte) 0;
  }
  bufferTextoIndice = 0;
  textoLongitud = 0;
  textoMascara = 0;
}


void imprimirInicioTexto(int tipo, int instruccion) {
  bufferTextoLimpiar();
  escribirCaracterEnTexto (caracterDeInicio);
  escribirNumeroIntEnTexto (tipo);
  escribirNumeroIntEnTexto (instruccion);
  bufferTextoIndice += 2;
}


void imprimirFinalTexto() {

  int indiceAux = bufferTextoIndice;

  bufferTextoIndice = 13;
  escribirNumeroIntEnTexto (textoLongitud + 4);

  bufferTextoIndice = indiceAux;
  escribirByteEnTexto(textoMascara);
  //bufferTexto[bufferTextoIndice++] = '\n';

  escribirCaracterEnTexto (caracterDeFin);
  reImprimirEnPuerto();
}

void reImprimirEnPuerto() {

  Serial.write(bufferTexto, bufferTextoIndice);
}

void escribirNumeroFloatEnTexto (float numeroFloat) {
  byte *arregloByte;
  arregloByte = (byte*) &numeroFloat;
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[0];
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[1];
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[2];
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[3];

  textoLongitud += 4;
  textoMascara ^= (byte) arregloByte[0];
  textoMascara ^= (byte) arregloByte[1];
  textoMascara ^= (byte) arregloByte[2];
  textoMascara ^= (byte) arregloByte[3];
}

void escribirNumeroLongEnTexto (long numeroLong) {
  byte *arregloByte;
  arregloByte = (byte*) &numeroLong;

  bufferTexto[bufferTextoIndice++] = (char) arregloByte[0];
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[1];
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[2];
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[3];

  textoLongitud += 4;
  textoMascara ^= (byte) arregloByte[0];
  textoMascara ^= (byte) arregloByte[1];
  textoMascara ^= (byte) arregloByte[2];
  textoMascara ^= (byte) arregloByte[3];
}

void escribirNumeroIntEnTexto (int numeroInt) {
  byte *arregloByte;
  arregloByte = (byte*) &numeroInt;

  bufferTexto[bufferTextoIndice++] = (char) arregloByte[0];
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[1];
  textoLongitud += 2;
  textoMascara ^= (byte) arregloByte[0];
  textoMascara ^= (byte) arregloByte[1];
}

void escribirByteEnTexto (byte numeroByte) {
  byte *arregloByte;
  arregloByte = (byte*) &numeroByte;
  bufferTexto[bufferTextoIndice++] = (char) arregloByte[0];
  textoLongitud ++;
  textoMascara ^= (byte) arregloByte[0];
}

void escribirCaracterEnTexto(char caracter) {
  bufferTexto[bufferTextoIndice++] = (char) caracter;
  textoLongitud ++;
  textoMascara ^= (byte) caracter;
}

void escribirStringEnTexto (String cadena) {
  if (cadena.length() < 60) {
    for (int i = 0; i < cadena.length (); i++) {
      bufferTexto[bufferTextoIndice++] = (char) cadena.charAt(i);
      textoLongitud++;
      textoMascara ^= (byte) cadena.charAt(i);
    }
  }
}

void escribirStringEnTexto (char* cadena) {
  for (int i = 0; cadena[i] != '\0' && i < 20; i++) {
    bufferTexto[bufferTextoIndice++] = cadena [i];
    textoLongitud++;
    textoMascara ^= (byte) cadena[i];
  }
}




void escribirByteEnMemoria (int direccion, byte numero) {
  EEPROM.writeByte (direccion, numero);
  EEPROM.commit();
}

byte leerByteEnMemoria (int direccion) {
  return EEPROM.readByte(direccion);
}
