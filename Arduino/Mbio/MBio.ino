#define ALARMA_ENCHUFE_TR_100_DESCONECTADO 1

#include <SPI.h>
#include <Ethernet.h>
#include <Math.h>

int analogAux_1;
#include <avr/wdt.h>
#include <PID_v1.h>
#include <EEPROMex.h>
#include <EEPROMVar.h>
#define DS1307_I2C_ADDRESS 0x68

#if defined(ARDUINO) && ARDUINO >= 100
  #define I2C_WRITE Wire.write 
  #define I2C_READ Wire.read
#else
  #define I2C_WRITE Wire.send 
  #define I2C_READ Wire.receive
#endif
#include <math.h>

#include "Wire.h"

byte registro1 = 0;
byte registro2 = 0;
byte registro3 = 0;
byte registro4 = 0;
byte registro5 = 0;
byte registro6 = 0;
byte registro7 = 0;
byte registro8 = 0;
byte registro9 = 0;

const byte registroOutNumFil = 5;
const byte registroOutNumCol = 8;
byte registroOut [registroOutNumFil];

byte DataIn_01 = 23;
byte SerialCtrl_01 = 25;
byte ClockIn_01 = 27;

byte DataIn_02 = 29;
byte SerialCtrl_02 = 31;
byte ClockIn_02 = 33;

byte DataIn_03 = 35;
byte SerialCtrl_03 = 37;
byte ClockIn_03 = 39;

byte DataIn_04 = 41;
byte SerialCtrl_04 = 43;
byte ClockIn_04 = 45;

byte DataIn_05 = 11;
byte SerialCtrl_05 = 13;
byte ClockIn_05 = 12;



byte ClockOut_01 = 22;
byte LatchCtrl_01 = 24;
byte DataOut_01 = 26;

byte ClockOut_02 = 28;
byte LatchCtrl_02 = 30;
byte DataOut_02 = 32;


byte ClockOut_03 = 10;
byte LatchCtrl_03 = 8;
byte DataOut_03 = 9;


byte Analog_Enable = 36;
byte Analog_S0 = 38;
byte Analog_S1 = 40;
byte Analog_S2 = 44;

int habilitarEntradaAnalogica (byte);
void actualizarSalidas();
byte entradas [] = {7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8, 23, 22, 21, 20, 19, 18, 17, 16, 31, 30, 29, 28, 27, 26, 25, 24, 39, 38, 37 ,36, 35, 34, 33, 32, 47, 46, 45, 44, 43, 42, 41, 40, 55, 54, 53, 52, 51, 50, 49, 48, 63, 62, 61, 60, 59, 58, 57, 56, 71, 70, 69, 68, 67, 66, 65, 64};  // original

const int numeroDeX = 80;
byte X [numeroDeX];

const int numeroDeY = 80;
byte Y[numeroDeY];

const int numeroDeM = 80;
byte M [numeroDeM];

const int numeroDeRC = 80;
byte RC [numeroDeRC];

const int numeroDeCMP = 16;
byte CMP[numeroDeCMP];

const int numeroDeZ = 8;
byte Z[numeroDeZ];

void imprimirDatosDigitales (int);
void imprimirBanderas (int);
void establecerBanderaBinaria(int);
void obtenerBanderaBinaria (int);

void imprimirDatosDigitales_2 (int);


int indice = 0; 
const int numeroDeLecturas = 1;
const int numeroDeSensores = 23;

int const numeroMaximoVA = 161; 
float VA[numeroMaximoVA];

void leerVariablesAnalogicasDeEEPROM (int);

void imprimirDatosAnalogicos (int);
void enviarDatosCanalAnalogico (int);
void enviarTodosDatosCanalAnalogico (int);
void modificarConfiguracionCanalAnalogico (int);

#define TA150_h 0.42
float TA150_x = 0;
float TA150_Vol = 0;

#define TA240_h 1.180
float TA240_x;
float TA240_Vol;

#define TA810_h 1.180
float TA810_x;
float TA810_Vol;

#define PRESION_RHO 920
#define PRESION_G 9.78
#define PRESION_H 4.8
#define PRESION_AREA 14.259784
float RB200_Vol = 0;


const int numeroDeTemporizadores = 48;
struct temporizador {
    byte entrada;
    byte salida;
    unsigned long tiempo;
    unsigned long tiempoActual;
} TON[numeroDeTemporizadores];
struct temporizadorAux {
    byte bandera;
    unsigned long tiempo_Aux1;
    unsigned long tiempo_Aux2;
} TON_Aux[numeroDeTemporizadores];

void leerTemporizadoresDeEEPROM  (int);
void configurarTemporizador (int);
void imprimirTemporizadores (int);
void actualizarTemporizador (int);

const int numeroDeContadores = 16;

struct contador {
    byte entrada;
    byte salida;
    byte habilitar;
    int cuentaActual;
    int cuentaMaxima;
    byte aux1;
    byte aux2;
} C[numeroDeContadores];

void actualizarContador (byte);

void leerContadoresDeEEPROM  (int);
void actualizarContador (byte);
void configurarContador (int);
void imprimirContadores (int);
void cambiarCuenta (byte, byte);


const int numeroDePID = 5;
struct controladorPID {
    float SetPoint;
    float Kp;
    float Kd;
    float Ki;
    float Input;
    float Output;
} PID_[numeroDePID];

struct tiempoPID {
    unsigned long tiempoTotal;
    unsigned long tiempoDeInicio;
    byte salidaBinaria;
} PID_tiempo [numeroDePID];

PID PID_00(&PID_[0].Input, &PID_[0].Output, &PID_[0].SetPoint, PID_[0].Kp, PID_[0].Ki, PID_[0].Kd, DIRECT);
PID PID_01(&PID_[1].Input, &PID_[1].Output, &PID_[1].SetPoint, PID_[1].Kp, PID_[1].Ki, PID_[1].Kd, DIRECT);
PID PID_02(&PID_[2].Input, &PID_[2].Output, &PID_[2].SetPoint, PID_[2].Kp, PID_[2].Ki, PID_[2].Kd, DIRECT);
PID PID_03(&PID_[3].Input, &PID_[3].Output, &PID_[3].SetPoint, PID_[3].Kp, PID_[3].Ki, PID_[3].Kd, DIRECT);
PID PID_04(&PID_[4].Input, &PID_[4].Output, &PID_[4].SetPoint, PID_[4].Kp, PID_[4].Ki, PID_[4].Kd, DIRECT);

void leerPIDDeEEPROM (int);
void configurarPID (int);
void imprimirValoresPID (int);

const int numeroDeAlarmas = 15;

typedef struct alar {
    float valor;
    byte estado;
    byte apagar;
};

struct alarma {
  alar AHH;
  alar AH;
  alar AL;
  alar ALL;
} alarmas[numeroDeAlarmas];

void leerAlarmasDeEEPROM (int);
void configurarAlarmas (int);
void imprimirAlarmas (int);
void actualizarAlarmas (float , int);
float Kalman_A = 1;
float Kalman_B = 0;
float Kalman_C = 1;

#define KALMAN_TEMPERATURA 0
#define KALMAN_NIVEL 1
#define KALMAN_PRESION 2
#define KALMAN_FLUJO 3
#define KALMAN_VOLUMEN 4
#define KALMAN_DEFAULT 5
float des_obs_3 [] = {2, 3, 2, 2, 3, 2};
float des_estado_3 [] = {0.01, 0.5, 0.1, 0.01, 1, 0.01};

float var_obs_3 [6];
float var_estado_3 [6];

const int numeroDeFiltroKalman = numeroDeSensores;
struct filtroKalman {
    float P_antes; 
    float X_antes;
    float P_k;
    float X_k;
    float K_k;
    float Y_k;
} kalman[numeroDeFiltroKalman];
int kalmanInicio = 0;

void filtroKalmanCalcular_3 (int, int);

void leerRegistro (byte, byte, byte, byte*);
void leerRegistro (byte, byte, byte, byte*, byte* );
void actualizarEntradas (byte, byte, byte, byte, byte, byte, byte, byte);

void imprimirEntradas ();
byte segundo = 0;
byte minuto = 0;
byte hora = 0;
byte diaDeLaSemana = 0;
byte diaDelMes = 0;
byte mes = 0;
byte anio = 0;
byte zero ;

byte decToBcd(byte);
byte bcdToDec(byte);
void setDateDs1307();

void escribirMySerial ();
void verificarHora ();

void imprimirFechaByte (int);
void establecerFechaByte (int);
void actualizarFecha (void);

const byte numeroDeDias = 8;
long horaActivacion[numeroDeDias ];// = 0;
long horaDesactivacion[numeroDeDias ];// = 0;
long horaActual = 0;

byte numeroDeMaximoDeActivaciones = 3;
byte numeroDeActivaciones[6];

void imprimirHorario (int);

#define EEPROM_SENSORES 0
#define EEPROM_TEMPORIZADORES 1
#define EEPROM_CONTADORES 2
#define EEPROM_PID 3
#define EEPROM_ALARMAS 4
#define EEPROM_HORA 5
#define EEPROM_PULSOS 6
#define EEPROM_BANDERA 7
#define EEPROM_MPU 8

String texto;
const int memoriaBase = 0;

void escribirByteEnMemoria (int, byte);
byte leerByteEnMemoria (int);
void escribirEnteroEnMemoria (int , int);
int leerEnteroEnMemoria (int);
void escribirLongEnMemoria (int, long);
long leerLongEnMemoria (int);
void escribirFloatEnMemoria (int , float);
float leerFloatEnMemoria (int);

void obtenerConfiguracion (int, int);
int obtenerDireccionMemoriaEEPROM (int);

const int numeroDePulsos = 5;
const int numeroDeBanderas = 2;
#define INSTRUCCION_CONF_FECHAC 110
#define INSTRUCCION_CONF_DIGITAL 111
#define INSTRUCCION_CONF_ANALOG 112
#define INSTRUCCION_CONF_ANALOG_T 113
#define INSTRUCCION_CONF_TEMPO 114
#define INSTRUCCION_CONF_CONTA 115
#define INSTRUCCION_CONF_PID 116
#define INSTRUCCION_CONF_FABRICA 100
#define INSTRUCCION_VALINI 1
#define INSTRUCCION_ANACONF 10
#define INSTRUCCION_DIGIO 22
#define INSTRUCCION_ALARC 24
#define INSTRUCCION_MPUCON 27
#define INSTRUCCION_HORASC 30
#define INSTRUCCION_VERSION 31
#define INSTRUCCION_COMU 32 
#define INSTRUCCION_INFOR 46
#define INSTRUCCION_INIMPU 47
#define INSTRUCCION_IMPRIMIR_DIGITAL 10
#define INSTRUCCION_IMPRIMIR_DIGITAL_2 11
#define INSTRUCCION_IMPRIMIR_ANALOG 12
#define INSTRUCCION_IMPRIMIR_ANALOG_2 13
#define INSTRUCCION_IMPRIMIR_ANALOG_CANAL_TODOS 14
#define INSTRUCCION_IMPRIMIR_ANALOG_CANAL_TODOS_2 15
#define INSTRUCCION_IMPRIMIR_TEMPORIZADORES_1 21
#define INSTRUCCION_IMPRIMIR_TEMPORIZADORES_2 22
#define INSTRUCCION_IMPRIMIR_TEMPORIZADORES_3 23
#define INSTRUCCION_IMPRIMIR_TEMPORIZADORES_4 24
#define INSTRUCCION_IMPRIMIR_TEMPORIZADORES_5 25
#define INSTRUCCION_IMPRIMIR_TEMPORIZADORES_6 26
#define INSTRUCCION_IMPRIMIR_PID_1 31
#define INSTRUCCION_IMPRIMIR_PID_2 32
#define INSTRUCCION_IMPRIMIR_PID_3 33
#define INSTRUCCION_IMPRIMIR_PID_4 34
#define INSTRUCCION_IMPRIMIR_CONTADORES_1 39
#define INSTRUCCION_IMPRIMIR_FECHA 40
#define INSTRUCCION_IMPRIMIR_ALARMA_1 51
#define INSTRUCCION_IMPRIMIR_ALARMA_2 52
#define INSTRUCCION_IMPRIMIR_ALARMA_3 53
#define INSTRUCCION_IMPRIMIR_ALARMA_4 54
#define INSTRUCCION_LEIDO_DE_EEPROM_ANALOG 61
#define INSTRUCCION_LEIDO_DE_EEPROM_TEMPO 62
#define INSTRUCCION_LEIDO_DE_EEPROM_CONTA 63
#define INSTRUCCION_LEIDO_DE_EEPROM_PID 64
#define INSTRUCCION_LEIDO_DE_EEPROM_ALARMA 65
#define INSTRUCCION_LEIDO_DE_EEPROM_HORARIO 66
#define SISTEMA_REINICIADO 160
#define INSTRUCCION_WATCHDOG 161
#define INSTRUCCION_RECIBIDA_CONF_FECHA 70
#define INSTRUCCION_RECIBIDA_CONF_DIGITAL 71
#define INSTRUCCION_RECIBIDA_CONF_ANALOG 73
#define INSTRUCCION_RECIBIDA_CONF_TEMPO 75
#define INSTRUCCION_RECIBIDA_CONF_CONTA 77
#define INSTRUCCION_RECIBIDA_CONF_PID 79
#define INSTRUCCION_RECIBIDA_CONF_ALARMA 81
#define INSTRUCCION_RECIBIDA_CONF_HORARIO 83
#define INSTRUCCION_RECIBIDA_CONF_BANDERA 85
#define INSTRUCCION_RECIBIDA_CONF_FABRICA 86
#define INSTRUCCION_IMPRIMIR_ANALOG_CANAL 13
#define INSTRUCCION_IMPRIMIR_HORARIO 18
#define INSTRUCCION_IMPRIMIR_BANDERA 45
#define INSTRUCCION_RECIBIDO 56
#define INSTRUCCION_MOSTRAR_BITS 57
#define INSTRUCCION_MOSTRAR_VERSION 58
#define INSTRUCCION_FECHAC 28
#define INSTRUCCION_ANAPARA 16

byte comuInstruccion1 = 0;
byte comuInstruccion2 = 0;
byte comuInstruccion3 = 0;
byte comuInstruccion4 = 0;

const int  bufferIndiceMaximo = 90;
byte bufferLectura [bufferIndiceMaximo];
byte bufferLectura1 [bufferIndiceMaximo]; 
byte bufferLectura2 [bufferIndiceMaximo]; 
byte bufferLectura3 [bufferIndiceMaximo]; 

int bufferIndice = 0;
int bufferIndice1 = 0;
int bufferIndice2 = 0;
int bufferIndice3 = 0;

char caracterDeFin = '*';
char caracterDeFin2 = '&';
char caracterDeInicio2 = '+';
char caracterDeInicio = '_';

byte bufferUltimaInstruccionByte [bufferIndiceMaximo];
byte bufferUltimaInstruccionByte1[bufferIndiceMaximo];
byte bufferUltimaInstruccionByte2[bufferIndiceMaximo];
byte bufferUltimaInstruccionByte3[bufferIndiceMaximo];
int tamanioBufferUltimaInstruccion = 0;
int tamanioBufferUltimaInstruccion1 = 0;
int tamanioBufferUltimaInstruccion2 = 0;
int tamanioBufferUltimaInstruccion3 = 0;

void colocarEnBufferDatosSerial(void);
void colocarEnBufferDatosSerial1(void);
void colocarEnBufferDatosSerial2(void);
void colocarEnBufferDatosSerial3(void);

void obtenerBufferInstruccion  (int*, byte*, int);
void leerInstruccionesDeBufferSerial(byte* , int* , byte*, int*, int);

int validarInformacionTrama (int, byte*, int);
int validarInformacionTrama (int);
int obtenerListaDeInstrucciones (byte*, int);
int obtenerListaDeInstrucciones (int);

void obtenerBuffer (int*, byte*, int);

void leerDatosSerial();
int  leerSerial (byte);      

byte imprimirNumPuerto = 0;
byte indiceDeInstrucciones = 0;
const byte indiceMaximoDeInstrucciones = 6;

const int bufferTextoMaximo = 170;
byte bufferTexto [bufferTextoMaximo];
int bufferTextoIndice = 0;

char textoOrigen [] = { 'M', 'B', 'i', 'o', '\0'};
long textoSecuencia = 0;
long textoDestino = 4294967295;
int textoInstruccion  = 0;
byte textoMascara = 0;
int textoLongitud = 0;

unsigned long tiempo_ultimo_01 = 0;
const unsigned long tiempo_intervalo_01 = 300;

unsigned long tiempo_ultimo_02 = 0;
const unsigned long tiempo_intervalo_02 = 200;

unsigned long tiempo_ultimo_03 = 0;
const unsigned long tiempo_intervalo_03 = 8;
byte tiempo_indice_03 = 0;

unsigned long tiempo_ultimo_04 = 0;
const unsigned long tiempo_intervalo_04 = 200;
byte tiempo_indice_04 = 0;

unsigned long tiempo_ultimo_05 = 0;
const unsigned long tiempo_intervalo_05 = 1000;
byte tiempo_indice_05 = 0;

unsigned long tiempo_ultimo_06 = 0;
const unsigned long tiempo_intervalo_06 = 1000;
byte tiempo_indice_06 = 0;

unsigned long tiempo_ultimo_07 = 0;
const unsigned long tiempo_intervalo_08 = 1000;
byte tiempo_indice_07 = 0;

#define PUERTO_SERIAL_0 0
#define PUERTO_SERIAL_1 1
#define PUERTO_SERIAL_2 2
#define PUERTO_SERIAL_3 3

void imprimirVersion (int);
void enviarInformacion (int);
void verificarComunicacion (int);
void imprimirInicioTexto2(int);
void imprimirFinalTexto2(int);
void reImprimirEnPuerto(int);

void bufferTextoLimpiar (void);

void imprimirInformacionBasura(int);

void leerDatosSerial(void);
void leerDatosSerial1(void);
void leerDatosSerial2(void);
void leerDatosSerial3(void);

float calcularPendiente (float, float, float, float );
float calcularOrdenada (float, float, float);
float calcularOrdenada (float, float, float, float);

long obtenerLongDeArregloByte (byte*);
float obtenerFloatDeArregloByte (byte*);
float obtenerFloatDeString (String );
int obtenerIntDeArregloByte (byte*);
byte obtenerByteDeArregloByte (byte*);
void obtenerBitsDeArregloByte (byte*, int*, int);

byte decToBcd(byte);
byte bcdToDec(byte);
String obtenerCadena (byte[], int, int);

void escribirNumeroFloatEnTexto (float);
void escribirNumeroLongEnTexto (long);
void escribirNumeroIntEnTexto (int);
void escribirByteEnTexto (byte);
void escribirCaracterEnTexto(char);
void escribirStringEnTexto (char *);
void escribirStringEnTexto (String);
void imprimirCadena (int, byte*, int);

byte direccion = 0;


float PID_AUX_0 = 0;

byte indiceVariables = 0;

void setup() {

    for (int i = 0; i < 6; i++ ) {
        var_obs_3 [i] = des_obs_3 [i] * des_obs_3 [i];
        var_estado_3 [i]= des_estado_3 [i]* des_estado_3 [i];
    }  
    wdt_disable();
    Serial.begin(19200);  
    Serial1.begin(19200);
    Serial2.begin(19200);
    Serial3.begin(9600);

    imprimirNumPuerto = PUERTO_SERIAL_2;
    EEPROM.setMaxAllowedWrites (1024);
    EEPROM.setMemPool(0, EEPROMSizeMega);
    

    Wire.begin();
    zero=0x00;  

    pinMode (DataIn_01, INPUT);
    pinMode (SerialCtrl_01, OUTPUT);
    pinMode (ClockIn_01, OUTPUT);

    pinMode (DataIn_02, INPUT);
    pinMode (SerialCtrl_02, OUTPUT);
    pinMode (ClockIn_02, OUTPUT);

    pinMode (DataIn_03, INPUT);
    pinMode (SerialCtrl_03, OUTPUT);
    pinMode (ClockIn_03, OUTPUT);

    pinMode (DataIn_04, INPUT);
    pinMode (SerialCtrl_04, OUTPUT);
    pinMode (ClockIn_04, OUTPUT);

    pinMode (DataIn_05, INPUT);
    pinMode (SerialCtrl_05, OUTPUT);
    pinMode (ClockIn_05, OUTPUT);


    pinMode(DataOut_01, OUTPUT);
    pinMode(LatchCtrl_01, OUTPUT);
    pinMode(ClockOut_01, OUTPUT);

    pinMode(DataOut_02, OUTPUT);
    pinMode(LatchCtrl_02, OUTPUT);
    pinMode(ClockOut_02, OUTPUT);

    pinMode(DataOut_03, OUTPUT);
    pinMode(LatchCtrl_03, OUTPUT);
    pinMode(ClockOut_03, OUTPUT);

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    
    pinMode(Analog_Enable, OUTPUT);
    pinMode(Analog_S0, OUTPUT);
    pinMode(Analog_S1, OUTPUT);
    pinMode(Analog_S2, OUTPUT);
    
    for (int i = 0; i < numeroMaximoVA; i++) {
        VA[i]= 0;
    }

    for (int i = 0; i < numeroDeFiltroKalman; i++) {
        kalman[i].P_k = 0;
        kalman[i].X_k = 0;
        kalman[i].K_k = 0;
        kalman[i].Y_k = 0;
    }

    for (int i = 0; i < 40; i++) {
      RC [i] = 0;
    }

    for (int i = 0; i < numeroDeDias; i++) {
         horaActivacion[i] = 0;
         horaDesactivacion[i] = 0;
     }


    obtenerConfiguracion (PUERTO_SERIAL_1, EEPROM_SENSORES);          // obtiene la configuracion de los sensores
    obtenerConfiguracion (PUERTO_SERIAL_1, EEPROM_TEMPORIZADORES);    // obtiene la configuracion de los temporizadores
    obtenerConfiguracion (PUERTO_SERIAL_1, EEPROM_CONTADORES);        // obtiene la configuración de los contadores 
    obtenerConfiguracion (PUERTO_SERIAL_1, EEPROM_PID);               // otiene la configuracion de PID
    obtenerConfiguracion (PUERTO_SERIAL_1, EEPROM_ALARMAS);           // obtiene la configuracion de las alarmas

    M[6] = 1;
    M[14] = 1;
    M[23] = 1;
//    M[10] = 1;
    M[43] = 1;

    PID_tiempo [0].tiempoTotal = 180000;
    PID_tiempo [0].tiempoDeInicio = millis();
    PID_tiempo [0].salidaBinaria = 0;

    PID_00.SetOutputLimits(0, PID_tiempo[0].tiempoTotal);
    PID_00.SetMode(AUTOMATIC);
    
    PID_tiempo [1].tiempoTotal = 360000;
    PID_tiempo [1].tiempoDeInicio = millis();
    PID_tiempo [1].salidaBinaria = 0;

    PID_01.SetOutputLimits(0, PID_tiempo[1].tiempoTotal);
    PID_01.SetMode(AUTOMATIC);
    
    PID_tiempo [2].tiempoTotal = 20000;
    PID_tiempo [2].tiempoDeInicio = millis();
    PID_tiempo [2].salidaBinaria = 0;

    PID_02.SetOutputLimits(0, PID_tiempo[2].tiempoTotal);
    PID_02.SetMode(AUTOMATIC);
    
    PID_tiempo [3].tiempoTotal = 20000;
    PID_tiempo [3].tiempoDeInicio = millis();
    PID_tiempo [3].salidaBinaria = 0;

    PID_03.SetOutputLimits(0, PID_tiempo[3].tiempoTotal);
    PID_03.SetMode(AUTOMATIC);
   
    PID_tiempo [4].tiempoTotal = 20000;
    PID_tiempo [4].tiempoDeInicio = millis();
    PID_tiempo [4].salidaBinaria = 0;

    PID_04.SetOutputLimits(0, PID_tiempo[4].tiempoTotal);
    PID_04.SetMode(AUTOMATIC);
    
    indiceDeInstrucciones = 0;

    actualizarFecha ();    
    imprimirInicioTexto2(SISTEMA_REINICIADO);
    escribirByteEnTexto(segundo);
    escribirByteEnTexto(minuto);
    escribirByteEnTexto(hora); 
    escribirByteEnTexto(diaDeLaSemana);
    escribirByteEnTexto(diaDelMes);
    escribirByteEnTexto(mes);
    escribirByteEnTexto(anio);
    imprimirFinalTexto2(imprimirNumPuerto);
    
    delay(1000);
    wdt_enable(WDTO_8S );
}



void loop (){



    byte aux = 0;

    leerRegistro (SerialCtrl_01, ClockIn_01, DataIn_01, &registro1, &registro2);
    leerRegistro (SerialCtrl_02, ClockIn_02, DataIn_02, &registro3, &registro4);
    leerRegistro (SerialCtrl_03, ClockIn_03, DataIn_03, &registro5, &registro6);
    leerRegistro (SerialCtrl_04, ClockIn_04, DataIn_04, &registro7, &registro8);
    
    leerRegistro (SerialCtrl_05, ClockIn_05, DataIn_05, &registro9);
    
    actualizarEntradas(registro1, registro2, registro3, registro4, registro5, registro6, registro7, registro8);
    
   if (X[32] == 0) {
       for (int i = 0; i < numeroDeX; i++) {
           if (i == 32) 
               continue; 
           else 
               X[i] = 0;
       }
   }
    
    imprimirEntradas();

    TON[0].entrada = !TON[0].salida;
    actualizarTemporizador(0);
 
    TON[1].entrada = !TON[1].salida;
    actualizarTemporizador(1);
    
    TON[2].entrada = !TON[2].salida;
    actualizarTemporizador(2);
    
    TON[3].entrada = !TON[3].salida;
    actualizarTemporizador(3);

    colocarEnBufferDatosSerial ();
    colocarEnBufferDatosSerial1();
    colocarEnBufferDatosSerial2();    

    if(millis() - tiempo_ultimo_04 > tiempo_intervalo_04) {
	tiempo_ultimo_04 = millis();
      actualizarFecha ();
    }

    if(millis() - tiempo_ultimo_01 > tiempo_intervalo_01) {
	tiempo_ultimo_01 = millis();

        if (indiceDeInstrucciones < indiceMaximoDeInstrucciones-1 ){
            indiceDeInstrucciones++;
        } else {
            indiceDeInstrucciones = 0;
        }

        switch (indiceDeInstrucciones) {
          case 0:
                  imprimirDatosDigitales (imprimirNumPuerto);
                  imprimirDatosAnalogicos (imprimirNumPuerto);
                  imprimirFechaByte (imprimirNumPuerto);
              break;
          
          case 1:
                  imprimirTemporizadores (imprimirNumPuerto);
              break;
              
          case 2: 
                  imprimirContadores (imprimirNumPuerto);
                  imprimirAlarmas (imprimirNumPuerto);
              break;
              
          case 3:
                  imprimirValoresPID (imprimirNumPuerto);
                  imprimirFechaByte (imprimirNumPuerto);
              break; 


          case 4:
                    leerInstruccionesDeBufferSerial(bufferLectura, &bufferIndice, bufferUltimaInstruccionByte, &tamanioBufferUltimaInstruccion, PUERTO_SERIAL_0);       
                    leerInstruccionesDeBufferSerial(bufferLectura1, &bufferIndice1, bufferUltimaInstruccionByte1, &tamanioBufferUltimaInstruccion1, PUERTO_SERIAL_1);                      
                    leerInstruccionesDeBufferSerial(bufferLectura2, &bufferIndice2, bufferUltimaInstruccionByte2, &tamanioBufferUltimaInstruccion2, PUERTO_SERIAL_2);  
              break;

          case 5:
                  imprimirFechaByte (imprimirNumPuerto);
                  imprimirInicioTexto2(INSTRUCCION_WATCHDOG);
                  imprimirFinalTexto2(imprimirNumPuerto);
              break;
        }    
    }
    RC[0] = X[0] & !M[0] & !RC[1] & !RC[2] & !RC[3] & !RC[4] & !RC[5];
    RC[1] = X[1] & !M[1];
    RC[2] = X[20]& !M[2];
    RC[3] = X[2] & !M[3];
    RC[4] = X[3] & !M[4];
    RC[5] = X[4] & !M[5];

    RC[28] = RC[1] | RC[2] | RC[3] | RC[4] | RC[5];
   
    TON[24].entrada = !RC[0] & RC[28];
    actualizarTemporizador (24);

    Y[38] = M[31];
    Y[39] = M[32];
    Y[40] = M[33] | RC[28] & !TON[24].salida;
    Y[42] = RC[28];
    Y[43] = M[36] | RC[28] & !TON[24].salida;
    RC[23] = RC[0] & X[8] & (M[23] | RC[23]) & !M[24] & !X[12];  

    TON[4].entrada  = RC[23];
    actualizarTemporizador (4);
    
    TON[5].entrada = RC[0] & TON[4].salida & !TON[6].salida;
    actualizarTemporizador (5);
  
    TON[6].entrada = TON[4].salida & TON[5].salida;
    actualizarTemporizador (6); 
    
    Y[2] = RC[0] & TON[4].salida & !TON[5].salida;
 
 RC[40] = RC[0] & (M[40] | RC[40] | X[18]) & !X[19] & !M[41];

 TON[18].entrada = RC[0] & RC[40];
 actualizarTemporizador (18);
 
 TON[19].entrada = RC[0] & RC[40] & TON[18].salida;
 actualizarTemporizador (19);
 
 TON[20].entrada = RC[0] & RC[40] & TON[19].salida;
 actualizarTemporizador (20);
 
 
 Y[3] = RC[0] & RC[40] & !TON[20].salida;
 Y[5] = RC[0] & TON[18].salida;
 Y[4] = RC[0] & TON[19].salida;

 RC[20] = RC[0] & (M[18] | RC[20] | X[29]) & !M[19] & !X[30] & X[5] & !X[10] & X[40] & !TON[25].salida & !RC[16];   
 
 TON[25].entrada = RC[0] & RC[20];
 actualizarTemporizador (25);
 
 TON[26].entrada = RC[0] & RC[20];
 actualizarTemporizador (26);
 
 TON[27].entrada = RC[0] & RC[20];
 actualizarTemporizador (27);
 

 Y[0] = RC[20] & TON[26].salida;

 RC[21] = RC[0] & (RC[20] & TON[27].salida | RC[21]) & !TON[28].salida;
 
 TON[28].entrada = RC[0] & !RC[20] & RC[21];
 actualizarTemporizador (28);
 
 Y[8] = RC[21];

 RC[18] = !X[54] & X[55];
 RC[19] = X[54] & !X[55];
 
 RC[16] = RC[0] & (M[16] | RC[16] | X[34]) & !M[17] & !X[15] & X[5] & !X[11] & !RC[20];
 Y[9] = RC[16];
 
 TON[33].entrada = RC[0] & RC[16];
 actualizarTemporizador (33);

 TON[17].entrada = RC[0] & RC[16]  & !X[47];
 actualizarTemporizador (17);


 RC[17]= RC[0] & TON[17].salida & RC[18] & !RC[19] & !X[11] & !TON[33].salida;
 Y[1] = RC[17] & RC[18] & !RC[19];

    RC[6] = RC[0]  & (M[6] | RC[6]) & !M[7] & RC[15];
    

    RC[15] = RC[0] & ( ( CMP[1] & RC[31] ) | RC[15] ) & !TON[15].salida;
    
    TON[15].entrada = RC[0] & (!CMP[1] | !RC[31])  & RC[15];
    actualizarTemporizador (15);


    TON[7].entrada = RC[0] & RC[6];
    actualizarTemporizador (7);
    
    RC[7] = RC[0] & TON[7].salida;
 
    Y[24] = RC[7] & ( !M[8] & !M[13]  |  M[13] );
    Y[25] = RC[7] & ( M[8] & !M[13] |  M[13] );
    
    TON[10].entrada = RC[7];
    actualizarTemporizador (10);
 
    RC[8] = RC[0] & M[9] & (TON[10].salida | RC[8]) & TON[10].salida;
 
    RC[9] = RC[0] & RC[8] & (RC[12] | RC[9]) & !TON[12].salida;
    TON[11].entrada = RC[8] & (RC[12] | RC[9]) & !TON[12].salida;
    actualizarTemporizador (11);
    RC[10] = RC[0] & RC[8] &  !RC[12];
    
    RC[11] = RC[0] & TON[11].salida & !RC[10];
    
    TON[12].entrada = RC[0] & RC[10];
    actualizarTemporizador (12);
    Y[23] = RC[9];
       
    TON[13].entrada = RC[8] & !TON[14].salida;                
    actualizarTemporizador (13);
    
    TON[14].entrada = RC[8] & TON[13].salida & TON[12].salida;
    actualizarTemporizador (14);
    
    RC[12] =  TON[13].entrada & !TON[13].salida;
   
    RC[13] = RC[0] & RC[30] & ((M[14] & alarmas[3].AH.estado )| RC[13]) & !M[15] & CMP[0] & !alarmas[3].AL.estado;  // RC[30] salida del PID_0
    
    
    TON[21].entrada = RC[13];                             // Tiempo de encendido
    actualizarTemporizador (21);
    
    TON[22].entrada = TON[21].salida & !TON[23].salida;   // Tiempo de encendido
    actualizarTemporizador (22);
  
    TON[23].entrada = TON[21].salida & TON[22].salida;    // Tiempo de apagado
    actualizarTemporizador (23); 
    
    Y[27] = RC[0] & TON[21].salida & !TON[22].salida;     //BC-350
    RC[14] = RC[0] & (M[10] | Y[1] | RC[14] )  & !M[11] & !TON[34].salida ;// & Y[1]
 
    TON[34].entrada = RC[0] & RC[14] & !Y[1]; //  Tiempo máximo de alimentación
    actualizarTemporizador (34);

    Y[28] = RC[14] & !M[12];

    Y[29] = RC[14] & M[12];

    RC[41] = !X[58] & X[59];
    RC[42] = X[58] & !X[59];


    RC[43]= RC[0] & (M[43] | RC[43]) & !M[44];
    
    RC[46] = RC[0] & RC[43] & (M[45] | alarmas[14].AH.estado | RC[46])  & !alarmas[14].AL.estado & !TON[31].salida & !M[46];
    TON[31].entrada = RC[0] & RC[43] &  RC[46] ;
    actualizarTemporizador (31);

    RC[44]= RC[0] & RC[43] & !RC[46]  & !TON[29].salida;
    TON[29].entrada = RC[0] & RC[43] & !RC[46];
    actualizarTemporizador (29);
    
    RC[45]= RC[0] & RC[43] & RC[46]  & !TON[30].salida;
    TON[30].entrada = RC[0] & RC[43] & RC[46];
    actualizarTemporizador (30);

    RC[47] = ((!RC[46] & !RC[44] & RC[41] ) | RC[47]) & !M[47];
 
    Y[11] = RC[44];
    Y[12] = RC[45];

    RC[56] = RC[0] & (RC[56] | M[56])  & !M[57];
    Y[6] = RC[56];
    
    RC[57] = RC[0] & (RC[57] | M[58])  & !M[59];
    Y[7] = RC[57];
    
    RC[58] = RC[0] & !X[52];

    RC[61] = !X[60] & X[61];
    RC[62] = X[60] & !X[61];
   
    RC[27] = RC[0] & (X[41] | RC[27] | M[28]) & !X[42] & !M[29] & !alarmas[12].AH.estado & !TON[32].salida;

    M[28] = 0;

    TON[32].entrada = RC[0] & (RC[61] | RC[27]);
    actualizarTemporizador (32);
 
    Y[13] = RC[27];
    Y[16] = RC[27];
    Y[15] = RC[27];

    RC[25] = RC[0] & (M[25] | RC[25] | (!M[51] & RC[51] ) | (RC[39]& TON[4].salida)) & !M[26];

    Y[30] = RC[25] & !M[27];    
    Y[31] = RC[25] & M[27];

    Y[32] = RC[0] & M[51] & !M[52] & !TON[44].salida  & !RC[51];
    Y[35] = Y[32];
    
    TON[44].entrada = RC[0] & M[51] & !M[52] & !TON[44].salida;
    actualizarTemporizador (44);
    
    if (TON[44].salida) {
          M[51] = 0;
          RC[51] = 1;
    }

    Y[33] = RC[0] & M[52] & !M[51] & !TON[45].salida & RC[51];
    Y[34] = Y[33];
    
    TON[45].entrada = RC[0] & M[52] & !M[51] & !TON[45].salida;
    actualizarTemporizador (45);
    
    if (TON[45].salida) {
          M[52] = 0;
          RC[51] = 0;
    }
    
    RC[38] = RC[0] & X[38] & X[39];
    RC[39] = RC[0] & !X[38] & !X[39];

    PID_[0].Input = VA[2 + 2 * numeroDeSensores]; 
    PID_00.Compute();

    PID_[1].Input = VA[1 + 2 * numeroDeSensores];
    PID_01.Compute();

    PID_[2].Input = VA[13 + 2 * numeroDeSensores];
    PID_02.Compute();

    PID_[3].Input = VA[10 + 2 * numeroDeSensores];
    PID_03.Compute();
    
    
    for (int i = 0; i < 4; i++) {
    
    if ((millis() - PID_tiempo[i].tiempoDeInicio )  > (PID_tiempo[i].tiempoTotal )) {
        PID_tiempo[i].tiempoDeInicio = millis();
    }
    
    if ((millis() - PID_tiempo[i].tiempoDeInicio ) < (PID_[i].Output) ) {
      
        PID_tiempo[i].salidaBinaria = 1;
    } else {
      
        PID_tiempo[i].salidaBinaria = 0;  
    }
    }


    RC[30] = PID_tiempo[0].salidaBinaria;
    RC[31] = PID_tiempo[1].salidaBinaria;
    RC[32] = PID_tiempo[2].salidaBinaria;
    RC[33] = PID_tiempo[3].salidaBinaria;

    actualizarSalidas();

    digitalWrite (2,Y[32]); // Y-32
    digitalWrite (3,Y[33]); // Y-33
    digitalWrite (4,Y[34]); // Y-34
    digitalWrite (5,Y[35]); // Y-35
    digitalWrite (6,Y[36]); // Y-36
    digitalWrite (7,Y[37]); // Y-37


    if(millis() - tiempo_ultimo_02 > tiempo_intervalo_02) {
	tiempo_ultimo_02 = millis();

        digitalWrite (LatchCtrl_01,0);
        digitalWrite (LatchCtrl_02,0);
        digitalWrite (LatchCtrl_03,0);        
            
        shiftOut(DataOut_01,ClockOut_01, MSBFIRST,(byte) registroOut[1]);
        shiftOut(DataOut_01,ClockOut_01, MSBFIRST,(byte) registroOut[0]);
    
        shiftOut(DataOut_02,ClockOut_02, MSBFIRST,(byte) registroOut[3]);
        shiftOut(DataOut_02,ClockOut_02, MSBFIRST,(byte) registroOut[2]);
        
        shiftOut(DataOut_03,ClockOut_03, MSBFIRST,(byte) registroOut[4]);
        
        digitalWrite (LatchCtrl_01,1);
        digitalWrite (LatchCtrl_02,1);    
        digitalWrite (LatchCtrl_03,1);

    }
      
      
    if((millis() - tiempo_ultimo_03) > tiempo_intervalo_03) {
	tiempo_ultimo_03 = millis();

               
        if (tiempo_indice_03 >= 8)
            tiempo_indice_03 = 0;
        else
            tiempo_indice_03++;
      
        switch (tiempo_indice_03) {
        
            case 0:
               break;
               
            case 1:
               VA[0] = analogRead(A0);
               VA[1] = analogRead(A1);
               VA[2] = analogRead(A2);
               VA[3] = analogRead(A3);
              break;
               
            case 2:
               VA[4] = analogRead(A4);
               VA[5] = analogRead(A5);
               VA[6] = analogRead(A6);
               VA[7] = analogRead(A7);
               break;
               
            case 3:
            
               VA[8] = analogRead(A8);

               if (X[64])
                 VA[9] = analogRead(A9);
               if (X[65])
                 VA[10] = analogRead(A10);
               if (X[66])               
                 VA[11] = analogRead(A11);
            
               break;
               
            case 4:
            
               if (X[67])  
                 VA[12] = analogRead(A12);
                 
               VA[13] = analogRead(A13);
               VA[14] = analogRead(A14);

               break;
               
            case 5:
            
               VA[15] = habilitarEntradaAnalogica (0);
               VA[16] = habilitarEntradaAnalogica (1);
               VA[17] = habilitarEntradaAnalogica (2);
               VA[18] = habilitarEntradaAnalogica (3);

               break;
               
            case 6:
            
               VA[19] = habilitarEntradaAnalogica (4);
               VA[20] = habilitarEntradaAnalogica (5);
               VA[21] = habilitarEntradaAnalogica (6);

               VA[22] = RB200_Vol;
               break;
               
            case 7:
                if (indice < numeroDeLecturas) {
            
                    if (indice == 0 ) {
                        for (int i = 0; i < numeroDeSensores; i++) {
                        VA[i + numeroDeSensores] =  0;
                        }      
                    }
                    for (int i = 0; i < numeroDeSensores; i++) {
                        VA[i + numeroDeSensores] =  VA[i] + VA[i + numeroDeSensores];
                    }
                    indice++;
                
                } else {
            
                    for (int i = 0; i < numeroDeSensores; i++) {
                        VA[i + numeroDeSensores] =  VA[i + numeroDeSensores]/(indice);
                    }

                    if (kalmanInicio  < 2) {
                        for (int i = 0; i < numeroDeFiltroKalman; i++) {                
                            kalman[i].P_k = VA[i + numeroDeSensores];
                        }
                        kalmanInicio++;
                      
                        
                    } else {
                        for (int i = 0; i < numeroDeFiltroKalman; i++) {                
                            kalman[i].Y_k = VA[i + numeroDeSensores];

                            switch (i) {
                                case 0:
                                case 1:
                                case 2:
                                case 3:
                                case 4:
                                case 5:
                                case 6:
                                case 7:
                                    filtroKalmanCalcular_3(i, KALMAN_TEMPERATURA);
                                    break;
                                case 8:    
                                    filtroKalmanCalcular_3(i, KALMAN_FLUJO);
                                    break;
                                case 9:
                                case 10:
                                case 11:
                                case 12:
                                    filtroKalmanCalcular_3(i, KALMAN_NIVEL);

                                    break;
                                    
                                case 13:
                                case 14:
                                    filtroKalmanCalcular_3(i, KALMAN_PRESION);

                                    break;


                                case 22:
                                    filtroKalmanCalcular_3(i, KALMAN_VOLUMEN);

                                    break;
                                    
                                default :

                                    filtroKalmanCalcular_3(i, KALMAN_DEFAULT);
                                    break;
                                
                            }

                            VA[i + 6 * numeroDeSensores] = Kalman_C * kalman[i].X_k;
                        }
                    }        
                    
                    if (M[20] == 0) {
                        for (int i = 0; i < numeroDeSensores; i++) {
                            VA[i + 2 * numeroDeSensores] = VA[i + 6 * numeroDeSensores]  * VA[i + 4 * numeroDeSensores] + VA[i + 5 * numeroDeSensores];
                            VA[i + 3 * numeroDeSensores] = VA[i + 6 * numeroDeSensores];
                        }               
                    } else {
                        for (int i = 0; i < numeroDeSensores; i++) {
                            VA[i + 2 * numeroDeSensores] = VA[i + numeroDeSensores]  * VA[i + 4 * numeroDeSensores] + VA[i + 5 * numeroDeSensores];
                            VA[i + 3 * numeroDeSensores] = VA[i + numeroDeSensores];
                        }
                    }
                    indice = 0;
            
                    TA150_x = VA[10 + 2 * numeroDeSensores] + TA150_h;
                 
                     if ( TA150_x > 0 )
                     {
                         if (TA150_x < TA150_h ) {
                              TA150_Vol = PI * (0.333333*pow(TA150_x,3) + 0.125*pow(TA150_x,2)+0.015625*TA150_x);
                         }
                         else {
                               TA150_Vol = PI * (0.333333*pow(TA150_h,3) + 0.125*pow(TA150_h,2)+ 0.015625*TA150_h + 0.3025 * (TA150_x-TA150_h));
                         }
                     } else 
                          TA150_Vol = 0;

                    TA240_x = VA[11 + 2 * numeroDeSensores];

                     if (TA240_x < TA240_h ) {
                          TA240_Vol = PI * TA240_x * 0.585225;
                     }
                     else {
                           TA240_Vol = PI * (TA240_x * 0.585225 + 0.02067 * pow((TA240_x-TA240_h),3) - 0.1905 * pow((TA240_x-TA240_h),2) + 0.585225* (TA240_x-TA240_h));
                     }

                    TA810_x = VA[12 + 2 * numeroDeSensores];

                     if (TA810_x < TA810_h ) {
                          TA810_Vol = PI * TA810_x * 0.585225;
                     }
                     else {
                           TA810_Vol = PI * (TA810_x * 0.585225 + 0.02067 * pow((TA810_x-TA810_h),3) - 0.1905 * pow((TA810_x-TA810_h),2) + 0.585225* (TA810_x-TA810_h));
                     }

                    if (M[70] == 0) {

                        VA[10 + 2 * numeroDeSensores] = TA150_Vol;
                        VA[11 + 2 * numeroDeSensores] = TA240_Vol;
                        VA[12 + 2 * numeroDeSensores] = TA810_Vol;
                    }
                    

                    RB200_Vol = ((VA[13 + 2 * numeroDeSensores] - VA[14 + 2 * numeroDeSensores])*102.04081/(PRESION_RHO * PRESION_G) +  PRESION_H) * PRESION_AREA;

                    actualizarAlarmas (TA150_Vol , 10);
                    
                    actualizarAlarmas (VA[0 + 2 * numeroDeSensores] , 0);
                                        
                    actualizarAlarmas (VA[1 + 2 * numeroDeSensores] , 1);
                                                            
                    actualizarAlarmas (VA[2 + 2 * numeroDeSensores] , 2);
                                                                                
                    actualizarAlarmas (VA[3 + 2 * numeroDeSensores] , 3);
                    
                    actualizarAlarmas (VA[10 + 2 * numeroDeSensores] , 10);
                    
                    actualizarAlarmas (VA[11 + 2 * numeroDeSensores] , 11);
                    
                    actualizarAlarmas (VA[12 + 2 * numeroDeSensores] , 12);

                    actualizarAlarmas (VA[13 + 2 * numeroDeSensores] , 13);

                    actualizarAlarmas (VA[14 + 2 * numeroDeSensores] , 14);

                }
               break;
            
        }
    }

    if (VA[3 + 2 * numeroDeSensores] > VA[2 + 2 * numeroDeSensores] + 3)
        CMP[0] = 1;
    else
        CMP[0] = 0;
        
    if (VA[2 + 2 * numeroDeSensores] > VA[1 + 2 * numeroDeSensores] + 3)
        CMP[1] = 1;
    else
        CMP[1] = 0;    

    if (VA[3 + 2 * numeroDeSensores] > 50)
        CMP[2] = 1;
    else
        CMP[2] = 0;    
        
    wdt_reset();


}

void imprimirDatosDigitales (int numPuerto) {
    byte numero = 0;
    
    int aux = 0;
  
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_DIGITAL);

    escribirNumeroIntEnTexto(numeroDeX);
    escribirNumeroIntEnTexto(numeroDeY);
    escribirNumeroIntEnTexto(numeroDeM);
    escribirNumeroIntEnTexto(numeroDeRC);
    escribirNumeroIntEnTexto(numeroDeCMP);

    aux = numeroDeX/8;
  
    for ( int j = 0; j < aux; j++) {
      numero = 0;
     
        for (int i =  0; i < 8 ; i++) {
            numero = numero | (X[i + 8*j] << i);
      }
      escribirByteEnTexto(numero);
    }

    aux = numeroDeY/8;
    
    for ( int j = 0; j < aux; j++) {
      numero = 0;
        for (int i =  0; i < 8 ; i++) {
            numero = numero | (Y[i + 8*j] << i);
      }
      escribirByteEnTexto(numero);
    }


    aux = numeroDeM/8;

    for ( int j = 0; j < aux; j++) { 
      numero = 0;
      
        for (int i =  0; i < 8 ; i++) {
            numero = numero | (M[i + 8*j] << i);
      }
      escribirByteEnTexto(numero);
    }  

    aux = numeroDeRC/8;
    for ( int j = 0; j < aux; j++) {
      numero = 0;
      
        for (int i =  0; i < 8 ; i++) {
            numero = numero | (RC[i + 8*j] << i);
      }
      escribirByteEnTexto(numero);
    }  

    aux = numeroDeCMP/8;
    for ( int j = 0; j < aux; j++) {
      numero = 0;
      
        for (int i =  0; i < 8 ; i++) {
            numero = numero | (CMP[i + 8*j] << i);
      }
      escribirByteEnTexto(numero);
    }  
    
    imprimirFinalTexto2(numPuerto);
}


void imprimirDatosDigitales_2 (int numPuerto) {
    byte numero = 0;
    
    int aux = 0;
  
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_DIGITAL_2);

    escribirNumeroIntEnTexto(numeroDeZ);

    aux = numeroDeZ/8;
  
    for ( int j = 0; j < aux; j++) {
      numero = 0;
     
        for (int i =  0; i < 8 ; i++) {
            numero = numero | (Z[i + 8*j] << i);
      }
      escribirByteEnTexto(numero);
    }

    imprimirFinalTexto2(numPuerto);
}


void imprimirBanderas (int numPuerto) {
    byte numero = 0;
    
    int aux = 0;
  

}


void establecerBanderaBinaria (int numPuerto, int opcion) {

    int inst = 0;
    int direccion = 0;
    int valor = 0;
    
    int* tamanio = 0;
    byte* cadena;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }


    imprimirCadena(*tamanio, cadena, numPuerto); 
    inst = obtenerIntDeArregloByte(cadena + 14);

    imprimirInicioTexto2(INSTRUCCION_RECIBIDA_CONF_BANDERA);

    direccion = obtenerIntDeArregloByte(cadena + 20);
    valor = (int) obtenerByteDeArregloByte(cadena  + 22);

    if (direccion < numeroDeX) {
        M [direccion] = valor;
        
        escribirStringEnTexto(F("M["));
        escribirStringEnTexto(String(direccion));
        escribirStringEnTexto(F("]= "));
        escribirStringEnTexto(String(M[direccion]));

        Serial.print("\n");
        Serial.print("M [");
        Serial.print(direccion);
        Serial.print ("]= ");
        Serial.print (valor);
        Serial.print("\n");
        
    } else {

        RC [direccion-numeroDeX] = valor;    

        escribirStringEnTexto(F("RC["));
        escribirStringEnTexto(String(direccion-numeroDeX));
        escribirStringEnTexto(F("]= "));
        escribirStringEnTexto(String(RC[direccion-numeroDeX]));

    }
    imprimirFinalTexto2(numPuerto);
}

 

void obtenerBanderaBinaria (int numPuerto, int opcion) {
    int direccion;
    int valor;

    int tamanio = 0;
    byte* cadena;

    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_BANDERA);
    
    obtenerBufferInstruccion  (&tamanio, cadena, numPuerto);;

    if (direccion < numeroDeX) {
        
        escribirStringEnTexto(F("X["));
        escribirStringEnTexto(String(direccion));
        escribirStringEnTexto(F("]= "));
        escribirStringEnTexto(String(X[direccion]));        


    } else {
        if (direccion < (numeroDeX + numeroDeRC)) {
            
            escribirStringEnTexto(F("RC["));
            escribirStringEnTexto(String(direccion-numeroDeX));
            escribirStringEnTexto(F("]= "));
             escribirStringEnTexto(String(RC[direccion-numeroDeX]));
        
        } else {
            if (direccion < (numeroDeX + numeroDeRC + numeroDeY)) {

                escribirStringEnTexto(F("Y["));
                escribirStringEnTexto(String(direccion-numeroDeX- numeroDeRC));
                escribirStringEnTexto(F("]= "));
                escribirStringEnTexto(String(Y [direccion-numeroDeX-numeroDeRC]));  
              
            } else {
              
                escribirStringEnTexto(F("CMP["));
                escribirStringEnTexto(String(direccion-numeroDeX- numeroDeRC - numeroDeY));
                escribirStringEnTexto(F("]= "));
                escribirStringEnTexto(String(CMP [direccion-numeroDeX-numeroDeRC-numeroDeY]));
              }
        }
    }
    imprimirFinalTexto2(numPuerto);
}


void leerVariablesAnalogicasDeEEPROM (int i) {
    VA[i + 4 * numeroDeSensores] = leerFloatEnMemoria (2 * i * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_SENSORES));
    VA[i + 5 * numeroDeSensores] = leerFloatEnMemoria ((2 * i + 1 ) * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_SENSORES));
}

void imprimirDatosAnalogicos (int numPuerto) {
 
    float numero = 0;
    int numeroDeValores = 0;

    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ANALOG);
    numeroDeValores =12;
    escribirNumeroIntEnTexto(numeroDeValores);    
    
    for (int i = 0; i < numeroDeValores; i++) {
        numero = VA[i + 2 * numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
        numero = VA[i + 3*numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
    }  
    imprimirFinalTexto2(numPuerto);
    
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ANALOG_2);
    numeroDeValores =11;
    escribirNumeroIntEnTexto(numeroDeValores);    
    for (int i = 12; i < numeroDeValores + 12; i++) {
        numero = VA[i + 2 * numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
        numero = VA[i + 3*numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
    } 
    imprimirFinalTexto2(numPuerto);
}



void enviarDatosCanalAnalogico (int numPuerto) {

    float numero = 0;
    int i = -1;

    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ANALOG_CANAL);
    escribirByteEnTexto(i);
    escribirStringEnTexto(F(""));
    numero  = VA[i + 4 * numeroDeSensores];
    escribirNumeroFloatEnTexto(numero);
  
    numero = VA[i + 5 * numeroDeSensores];
    escribirNumeroFloatEnTexto(numero);

    imprimirFinalTexto2(numPuerto);
}

void enviarTodosDatosCanalAnalogico (int numPuerto) {

    float numero = 0;
    int numeroEntero = -1;
    byte *arregloByte;
    int numeroDeValores = 0;

    numeroDeValores =12;
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ANALOG_CANAL_TODOS);
    escribirNumeroIntEnTexto(numeroDeValores);

    for (int i = 0; i < numeroDeValores; i++) {

        numero  = VA[i + 4 * numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
 
        numero = VA[i + 5 * numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
        }
    imprimirFinalTexto2(numPuerto);
    
    numeroDeValores =11;
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ANALOG_CANAL_TODOS_2);
    escribirNumeroIntEnTexto(numeroDeValores);

    for (int i = 12; i < numeroDeValores+12; i++) {

        numero  = VA[i + 4 * numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
 
        numero = VA[i + 5 * numeroDeSensores];
        escribirNumeroFloatEnTexto(numero);
        }
    imprimirFinalTexto2(numPuerto);
    
}
 
void modificarConfiguracionCanalAnalogico (int numPuerto) {

    float m = 0;
    float b = 0;

    byte cantidadDeSenalesAnalog = 0;
    byte direccion;
    long valor;

    int* tamanio = 0;
    byte* cadena;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }

    imprimirCadena(*tamanio, cadena, numPuerto);

    cantidadDeSenalesAnalog = obtenerByteDeArregloByte (cadena + 20);

    Serial.print("\nCantidad ");
    Serial.print(cantidadDeSenalesAnalog);

    for (int i = 0; i < cantidadDeSenalesAnalog;i++) {
        direccion = obtenerByteDeArregloByte (cadena + 20 + 9*i + 1);
        
        m = obtenerFloatDeArregloByte (cadena + 20 + 9*i + 2);
        b = obtenerFloatDeArregloByte (cadena + 20 + 9*i + 6);        


        Serial.print("\nVA[");
        Serial.print(direccion);
        Serial.print("] = ");
        Serial.print(m);
        Serial.print("   ");
        Serial.print(b);
        
        escribirFloatEnMemoria (2*direccion * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_SENSORES),m);
        escribirFloatEnMemoria ((2 * direccion +1) * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_SENSORES),b);

    }
    for (int i = 0; i < numeroDeSensores; i++) {
        leerVariablesAnalogicasDeEEPROM (i);
    }
  
    enviarTodosDatosCanalAnalogico (numPuerto);

}

void leerTemporizadoresDeEEPROM  (int i) {
    long num = 0;
    num =  leerLongEnMemoria (i * sizeof(long) + obtenerDireccionMemoriaEEPROM (EEPROM_TEMPORIZADORES));
    TON [i].tiempo = num;
}

void configurarTemporizador (int numPuerto) {
    byte cantidadDeTemporizadores = 0;
    byte direccion;
    long valor;
    
    int* tamanio = 0;
    byte* cadena;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }

    imprimirCadena(*tamanio, cadena, numPuerto);

    imprimirInicioTexto2(INSTRUCCION_RECIBIDA_CONF_TEMPO );
    imprimirFinalTexto2(numPuerto);
   
    cantidadDeTemporizadores = obtenerByteDeArregloByte (cadena + 20);

    for (int i = 0; i < cantidadDeTemporizadores;i++) {
        direccion = obtenerByteDeArregloByte (cadena + 20 + 5*i+ 1);
        valor = obtenerLongDeArregloByte (cadena + 20 + 5*i + 2);
        escribirLongEnMemoria (direccion * sizeof(long)  + obtenerDireccionMemoriaEEPROM (EEPROM_TEMPORIZADORES), valor);
    }
    for (int i = 0; i < numeroDeTemporizadores; i++) {
        leerTemporizadoresDeEEPROM(i);
    }
}

void imprimirTemporizadores (int numPuerto)
{

    int numeroInt;
    unsigned long numeroLong;
    temporizador *tempo;
    byte *arregloByte;
    int tamanio;
    
    int auxAlar_1;
    int auxAlar_2;

    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_TEMPORIZADORES_1);
    
    auxAlar_1 = 0;
    auxAlar_2 = 8;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & TON[j];
        tamanio = sizeof(temporizador);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto((byte)arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);
    
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_TEMPORIZADORES_2);
    auxAlar_1 = 8;
    auxAlar_2 = 16;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & TON[j];
        tamanio = sizeof(temporizador);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto((byte)arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);

    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_TEMPORIZADORES_3);   
    auxAlar_1 = 16;
    auxAlar_2 = 24;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & TON[j];
        tamanio = sizeof(temporizador);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto((byte)arregloByte[i]);
            
        }
    }
    imprimirFinalTexto2(numPuerto);

    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_TEMPORIZADORES_4);   
    auxAlar_1 = 24;
    auxAlar_2 = 32;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & TON[j];
        tamanio = sizeof(temporizador);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto((byte)arregloByte[i]);
        }
    }
    
    imprimirFinalTexto2(numPuerto);
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_TEMPORIZADORES_5);   
    auxAlar_1 = 32;
    auxAlar_2 = 40;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & TON[j];
        tamanio = sizeof(temporizador);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto((byte)arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);    
    
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_TEMPORIZADORES_6);   
    auxAlar_1 = 40;
    auxAlar_2 = 48;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & TON[j];
        tamanio = sizeof(temporizador);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto((byte)arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);    
}

void actualizarTemporizador (int i) {
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

void leerContadoresDeEEPROM  (int i) {
    long num = 0;
    num =  leerEnteroEnMemoria (i *sizeof(int) + obtenerDireccionMemoriaEEPROM (EEPROM_CONTADORES));
    
    C[i].cuentaMaxima = num;
}

void actualizarContador (byte numeroContador) {
   
    if (!C[numeroContador].habilitar) {
        C[numeroContador].salida = 0;
        C[numeroContador].cuentaActual = 0;
    } else {
      
        C[numeroContador].aux2 = C[numeroContador].aux1 & !C[numeroContador].entrada;
    
        if (C[numeroContador].aux2 & C[numeroContador].aux1) {
              C[numeroContador].cuentaActual++;
        }
    
        C[numeroContador].aux1 = C[numeroContador].entrada;
    
        if (C[numeroContador].cuentaActual >= C[numeroContador].cuentaMaxima ) {
            C[numeroContador].salida = 1;
        }
    }
}




void configurarContador (int numPuerto) {
  
    byte cantidadDeContadores = 0;
    byte direccion;
    long valor;


    int* tamanio = 0;
    byte* cadena;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }

    imprimirCadena(*tamanio, cadena, numPuerto);

    imprimirInicioTexto2(INSTRUCCION_RECIBIDA_CONF_CONTA );
    imprimirFinalTexto2(numPuerto);

    
    cantidadDeContadores = obtenerByteDeArregloByte (cadena + 20);

    for (int i = 0; i < cantidadDeContadores;i++) {
      
        direccion = obtenerByteDeArregloByte (cadena + 20 + 3*i+ 1);
        valor = obtenerLongDeArregloByte (cadena + 20 + 3*i + 2);

        escribirEnteroEnMemoria (direccion*sizeof(int) + obtenerDireccionMemoriaEEPROM (EEPROM_CONTADORES), valor);
    }
    for (int i = 0; i < numeroDeContadores; i++) {
        leerContadoresDeEEPROM(i);
    }
}

void imprimirContadores (int numPuerto)
{
    int numeroInt;
    byte *arregloByte;
    int tamanio;
    int aux;

    aux = numeroDeContadores;

    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_CONTADORES_1); 
    escribirNumeroIntEnTexto(numeroDeContadores);

    
    for (int j = 0; j < aux; j++) {
        arregloByte = (byte*) & C[j];
        tamanio = sizeof(contador) - 2;
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto((byte)arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);
}

void cambiarCuenta (byte numeroNuevo, byte numeroContador) {
  
    if  (numeroNuevo > 0) {
        C[numeroContador].cuentaMaxima = numeroNuevo;
    }
}

void leerPIDDeEEPROM (int i) {
    PID_[i].SetPoint = leerFloatEnMemoria ((0+i*4)* sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID));
    PID_[i].Kp = leerFloatEnMemoria ((1+i*4)* sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID));
    PID_[i].Kd = leerFloatEnMemoria ((2+i*4)* sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID));
    PID_[i].Ki = leerFloatEnMemoria ((3+i*4)* sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID));
    switch (i) {
        case 0:
            PID_00.SetTunings (PID_[i].Kp, PID_[i].Ki, PID_[i].Kd);
            break;
        
        case 1:
            PID_01.SetTunings (PID_[i].Kp, PID_[i].Ki, PID_[i].Kd);
            break;
            
        case 2:
            PID_02.SetTunings (PID_[i].Kp, PID_[i].Ki, PID_[i].Kd);
            break;

        case 3:
            PID_03.SetTunings (PID_[i].Kp, PID_[i].Ki, PID_[i].Kd);
            break;
            
        case 4:
            PID_04.SetTunings (PID_[i].Kp, PID_[i].Ki, PID_[i].Kd);
            break;
            
    }

}

void configurarPID (int numPuerto) {

    byte numeroPID;
    byte direccion;
    
    int* tamanio = 0;
    byte* cadena;
   
    float SetP;
    float Ki;
    float Kp;
    float Kd;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }
    
    imprimirInicioTexto2(INSTRUCCION_RECIBIDA_CONF_PID );
    imprimirFinalTexto2(numPuerto);
    

    numeroPID = obtenerByteDeArregloByte(cadena + 20);

    for (int i = 0; i < numeroPID;i++) {
        direccion = obtenerByteDeArregloByte (cadena + 21 + 17*i);
        SetP = obtenerFloatDeArregloByte (cadena + 21 + 1);
        Kp = obtenerFloatDeArregloByte (cadena + 21 + 4 + 1);
        Kd = obtenerFloatDeArregloByte (cadena + 21 + 8 + 1);
        Ki = obtenerFloatDeArregloByte (cadena + 2 + 12 + 1 );
    
        escribirFloatEnMemoria ((0 + 4 * direccion) * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID), SetP);
        escribirFloatEnMemoria ((1 + 4 * direccion) * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID), Kp);
        escribirFloatEnMemoria ((2 + 4 * direccion) * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID), Kd);
        escribirFloatEnMemoria ((3 + 4 * direccion) * sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID), Ki);    
    }
    
    for (int i = 0; i < numeroDePID; i++) {
        leerPIDDeEEPROM(i);
    }
 
}

void imprimirValoresPID (int numPuerto) {
    byte *arregloByte;
    int tamanio;
    
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_PID_1);

    for (int j = 0; j < numeroDePID; j++) {
        arregloByte = (byte*) & PID_[j];
        tamanio = sizeof(controladorPID);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto(arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);
}

void leerAlarmasDeEEPROM (int i) {

    alarmas[i].AHH.valor = leerFloatEnMemoria ((4 * i + 0 )*(sizeof(float) + 2 * sizeof(byte) ) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));
    alarmas[i].AHH.estado = leerByteEnMemoria ((4 * i + 0 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));            
    alarmas[i].AHH.apagar = leerByteEnMemoria ((4 * i + 0 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float) + sizeof(byte)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));

    alarmas[i].AH.valor = leerFloatEnMemoria ((4 * i + 1 )*(sizeof(float) + 2 * sizeof(byte) ) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));
    alarmas[i].AH.estado = leerByteEnMemoria ((4 * i + 1 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));            
    alarmas[i].AH.apagar = leerByteEnMemoria ((4 * i + 1 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float) + sizeof(byte)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));

    alarmas[i].AL.valor = leerFloatEnMemoria ((4 * i + 2 )*(sizeof(float) + 2 * sizeof(byte) ) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));
    alarmas[i].AL.estado = leerByteEnMemoria ((4 * i + 2 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));            
    alarmas[i].AL.apagar = leerByteEnMemoria ((4 * i + 2 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float) + sizeof(byte)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));

    alarmas[i].ALL.valor = leerFloatEnMemoria ((4 * i + 3 )*(sizeof(float) + 2 * sizeof(byte) ) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));
    alarmas[i].ALL.estado = leerByteEnMemoria ((4 * i + 3 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));            
    alarmas[i].ALL.apagar = leerByteEnMemoria ((4 * i + 3 )*(sizeof(float) + 2 * sizeof(byte)) + sizeof(float) + sizeof(byte)   + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));
}

void configurarAlarmas (int numPuerto) {

    int tamanio = 0;
    byte* cadena;

    byte numeroAlarma;

    float alarmaSup;
    float alarmaInf;
    float advertenciaSup;
    float advertenciaInf;

    imprimirInicioTexto2(INSTRUCCION_RECIBIDA_CONF_ALARMA );
    imprimirFinalTexto2(numPuerto);
    
    obtenerBufferInstruccion  (&tamanio, cadena, numPuerto);
   
    numeroAlarma = obtenerByteDeArregloByte(cadena + 8);

    alarmaSup = obtenerFloatDeArregloByte (cadena + 9);
    advertenciaSup = obtenerFloatDeArregloByte (cadena + 9 + 4);
    advertenciaInf = obtenerFloatDeArregloByte (cadena + 9 + 8);
    alarmaInf = obtenerFloatDeArregloByte (cadena + 9 + 12);

    escribirFloatEnMemoria ((0 + 4 * numeroAlarma) *sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS), alarmaSup); //
    escribirFloatEnMemoria ((1 + 4 * numeroAlarma) *sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS), advertenciaSup); //
    escribirFloatEnMemoria ((2 + 4 * numeroAlarma) *sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS), advertenciaInf); //
    escribirFloatEnMemoria ((3 + 4 * numeroAlarma) *sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS), alarmaInf); //
        
    for (int i = 0; i < numeroDeAlarmas; i++) {
        leerAlarmasDeEEPROM(i);
    }
  
}

void imprimirAlarmas (int numPuerto) {
    byte *arregloByte;
    int tamanio;
    int auxAlar_1;
    int auxAlar_2;
   
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ALARMA_1);

    auxAlar_1 = 0;
    auxAlar_2 = 4;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & alarmas[j];
        tamanio = sizeof(alarmas[j]);
        
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto(arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);
    
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ALARMA_2);

    auxAlar_1 = 4;
    auxAlar_2 = 8;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & alarmas[j];
        tamanio = sizeof(alarmas[j]);
        
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto(arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);
    
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ALARMA_3);

    auxAlar_1 = 8;
    auxAlar_2 = 12;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & alarmas[j];
        tamanio = sizeof(alarmas[j]);
        
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto(arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);
    
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_ALARMA_4);

    auxAlar_1 = 12;
    auxAlar_2 = 15;

    escribirNumeroIntEnTexto(auxAlar_1);
    escribirNumeroIntEnTexto(auxAlar_2);

    for (int j = auxAlar_1; j < auxAlar_2; j++) {
        arregloByte = (byte*) & alarmas[j];
        tamanio = sizeof(alarmas[j]);
        
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto(arregloByte[i]);
        }
    }
    imprimirFinalTexto2(numPuerto);
    
}



void actualizarAlarmas (float valor,  int indice) {
  
    if (valor > alarmas[indice].AHH.valor) {
        alarmas[indice].AHH.estado = 1;
    } else {
        alarmas[indice].AHH.estado = 0;
    }
    
    if (valor > alarmas[indice].AH.valor) {
        alarmas[indice].AH.estado = 1;
    } else {
        alarmas[indice].AH.estado = 0;
    }
    
    if (valor < alarmas[indice].AL.valor) {
        alarmas[indice].AL.estado = 1;
    } else {
        alarmas[indice].AL.estado = 0;
    }
   
    if (valor < alarmas[indice].ALL.valor) {
        alarmas[indice].ALL.estado = 1;
    } else {
        alarmas[indice].ALL.estado = 0;
    }
}
void filtroKalmanCalcular_3 (int i, int j) {
    kalman[i].X_antes = Kalman_A * kalman[i].X_k + Kalman_B;
    kalman[i].P_antes = Kalman_A*kalman[i].P_k*Kalman_A + var_estado_3[j];
    
    kalman[i].K_k = kalman[i].P_antes / (kalman[i].P_antes + var_obs_3[j]);
    kalman[i].X_k = kalman[i].X_antes + kalman[i].K_k*(kalman[i].Y_k-kalman[i].X_antes);
    kalman[i].P_k = (1-kalman[i].K_k) * kalman[i].P_antes;
}



void leerRegistro (byte serialControl, byte clockInPin, byte dataInPin, byte* registro1)
{
    byte dato = 0;
    byte registro;

    digitalWrite (serialControl, 0);
    
    registro = 0;
    for (int i = 7; i>= 0; i--) 
    {
        digitalWrite (clockInPin, 0);
        delayMicroseconds (2);
        dato = digitalRead (dataInPin);
        if (dato) {
            registro = registro | (1 << i);
        } else {

        }
        digitalWrite (clockInPin, 1);
        *registro1 = registro;
    }


    digitalWrite (serialControl, 1);
}



void leerRegistro (byte serialControl, byte clockInPin, byte dataInPin, byte* registro1, byte* registro2)
{
    byte dato = 0;
    byte registro;

    digitalWrite (serialControl, 0);
    
    registro = 0;
    for (int i = 7; i>= 0; i--) 
    {
        digitalWrite (clockInPin, 0);
        delayMicroseconds (2);
        dato = digitalRead (dataInPin);
        if (dato) {
            registro = registro | (1 << i);
        } else {

        }
        digitalWrite (clockInPin, 1);
        *registro1 = registro;
    }

    registro = 0;
    for (int i = 7; i>= 0; i--) 
    {
        digitalWrite (clockInPin, 0);
        delayMicroseconds (2);
        dato = digitalRead (dataInPin);
        if (dato) {
            registro = registro | (1 << i);
        } else {

        }
        digitalWrite (clockInPin, 1);
        *registro2 = registro;
    }
    digitalWrite (serialControl, 1);
}



void actualizarEntradas (byte reg1, byte reg2, byte reg3, byte reg4, byte reg5, byte reg6, byte reg7, byte reg8) {
  
    byte aux;
    byte aux2;

    for (int j = 0; j < 9; j++) {

        switch (j) {
          case 0:
              aux = reg1;
              break;
          case 1:
              aux = reg2;
              break;
          case 2:
              aux = reg3;
              break;
          case 3:
              aux = reg4;
              break;
          case 4:
              aux = reg5;
              break;
          case 5:
              aux = reg6;
              break;
          case 6:
              aux = reg7;
              break;
          case 7:
              aux = reg8;
              break;

          case 8:
              aux = registro9;
              break;

        }
      
        for ( int i = 0; i <= 7; i++ ) {
          
            aux2 = (aux >> i) & 1;
            X[entradas[7 - i + j*8]] = aux2; 
        }
    }
}

void actualizarSalidas(){

  
    byte aux;
    for (int j = 0; j < 4; j++) {
        aux = 0;
        for ( int i = 0; i <= 7; i++ ) {
           aux = aux | ((Y[i + j*8] & 1) << i);
        }
        registroOut[j] = aux;
    }
    for (int j = 4; j < 5; j++) {
        aux = 0;
        for ( int i = 0; i <= 7; i++ ) {
           aux = aux | ((Y[i + j*8 + 6] & 1) << i);
        }
        registroOut[j] = aux;
    }
}
void imprimirEntradas ()
{
    Serial.print("\n");

    for (int j = 0; j < 8; j++) {

        for (int i = 0; i<= 7; i++)
            {
              Serial.print (X[i+j*8]); 
            }
        Serial.print (" ");
    }   
}
int habilitarEntradaAnalogica (byte entrada) {
      byte S0 = 0;
      byte S1 = 0;
      byte S2 = 0;
      int ent = 0;

      S0 = (entrada >> 0) & 1;
      S1 = (entrada >> 1) & 1;
      S2 = (entrada >> 2) & 1;

      digitalWrite (Analog_Enable, 0);  
      digitalWrite (Analog_S0, S0);
      digitalWrite (Analog_S1, S1);
      digitalWrite (Analog_S2, S2);

      ent = analogRead (A15);
      digitalWrite (Analog_Enable, 1);
      return ent;
}
void imprimirFechaByte (int numPuerto) {
  
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_FECHA);

    escribirByteEnTexto(segundo);
    escribirByteEnTexto(minuto);
    escribirByteEnTexto(hora); 
    escribirByteEnTexto(diaDeLaSemana);
    escribirByteEnTexto(diaDelMes);
    escribirByteEnTexto(mes);
    escribirByteEnTexto(anio);
    
    imprimirFinalTexto2(numPuerto);

}
  

void establecerFechaByte (int numPuerto){

    int* tamanio = 0;
    byte* cadena;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }

    imprimirCadena(*tamanio, cadena, numPuerto);

    imprimirInicioTexto2(INSTRUCCION_RECIBIDA_CONF_FECHA );
    imprimirFinalTexto2(numPuerto);
  
    segundo = cadena [20 + 0];  
    minuto = cadena [20 + 1];
    hora = cadena [20 + 2];
    diaDeLaSemana = cadena [20 + 3];
    diaDelMes = cadena [20 + 4];
    mes = cadena [20 + 5];
    anio = cadena [20 + 6]; 


   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   I2C_WRITE(zero);
   I2C_WRITE(decToBcd(segundo) & 0x7f);
   I2C_WRITE(decToBcd(minuto));
   I2C_WRITE(decToBcd(hora));

   I2C_WRITE(decToBcd(diaDeLaSemana));
   I2C_WRITE(decToBcd(diaDelMes));
   I2C_WRITE(decToBcd(mes));
   I2C_WRITE(decToBcd(anio));
   Wire.endTransmission();    
}
 
void setDateDs1307()                
{
 
   segundo = (byte) ((Serial.read() - 48) * 10 + (Serial.read() - 48));
   minuto = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   hora  = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   diaDeLaSemana = (byte) (Serial.read() - 48);
   diaDelMes = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   mes = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   anio= (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   I2C_WRITE(zero);
   I2C_WRITE(decToBcd(segundo) & 0x7f);
   I2C_WRITE(decToBcd(minuto));
   I2C_WRITE(decToBcd(hora));

   I2C_WRITE(decToBcd(diaDeLaSemana));
   I2C_WRITE(decToBcd(diaDelMes));
   I2C_WRITE(decToBcd(mes));
   I2C_WRITE(decToBcd(anio));
   Wire.endTransmission();
}
 

void actualizarFecha () {
    byte numero = 0;
    Wire.beginTransmission(DS1307_I2C_ADDRESS);
    I2C_WRITE(zero);
    Wire.endTransmission();
 
    Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

    segundo = bcdToDec(I2C_READ() & 0x7f);
    minuto = bcdToDec(I2C_READ());
    hora = bcdToDec(I2C_READ() & 0x3f);


    numero = bcdToDec(I2C_READ());
    if (numero >= 1 && numero <= 7) {
        diaDeLaSemana = numero;
    } else {
        diaDeLaSemana = 0;  
    }

    diaDelMes = bcdToDec(I2C_READ());
    mes = bcdToDec(I2C_READ());
    anio = bcdToDec(I2C_READ());
}
void imprimirHorario (int numPuerto)
{
    byte *arregloByte;
    int tamanio;
   
    imprimirInicioTexto2(INSTRUCCION_IMPRIMIR_HORARIO );

    escribirNumeroIntEnTexto(2);

    arregloByte = (byte*) &horaActual;
    tamanio = sizeof(long);
    for (int i = 0; i < tamanio; i++) {
        escribirByteEnTexto(arregloByte[i]);
    }


    for (int i = 0; i < numeroDeDias; i++) {
      
        arregloByte = (byte*) &horaActivacion[i];
        tamanio = sizeof(long);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto(arregloByte[i]);
        }

        arregloByte = (byte*) &horaDesactivacion[i];
        tamanio = sizeof(long);
        for (int i = 0; i < tamanio; i++) {
            escribirByteEnTexto(arregloByte[i]);
        }
    }

    imprimirFinalTexto2(numPuerto);
 }

int obtenerDireccionMemoriaEEPROM (int opcion)
{
int direccionInicial = 0;  
    switch (opcion) { 

        case EEPROM_SENSORES:
            direccionInicial = 0;
            break;
            
        case EEPROM_TEMPORIZADORES:
            direccionInicial = numeroDeSensores * 2 * sizeof(float) +  obtenerDireccionMemoriaEEPROM (EEPROM_SENSORES);
        break;
        
        case EEPROM_CONTADORES:
            direccionInicial = numeroDeTemporizadores * sizeof(long) + obtenerDireccionMemoriaEEPROM (EEPROM_TEMPORIZADORES);
        break;
        
        case EEPROM_PID:
            direccionInicial = numeroDeContadores * sizeof(int) + obtenerDireccionMemoriaEEPROM (EEPROM_CONTADORES);
        break;
        
        case EEPROM_ALARMAS:
            direccionInicial = numeroDePID * 4* sizeof(float) + obtenerDireccionMemoriaEEPROM (EEPROM_PID);
            break;

        case EEPROM_HORA:
            direccionInicial = numeroDeAlarmas * (4 * sizeof(float) + 2 * sizeof(byte)) + obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS);
            break;
    }
    return direccionInicial;
}

void escribirByteEnMemoria (int direccion, byte numero) {
    EEPROM.updateByte (direccion, numero);
}

byte leerByteEnMemoria (int direccion) {
    return EEPROM.readByte(direccion + memoriaBase);
}

void escribirEnteroEnMemoria (int direccion, int numero) {
    EEPROM.updateInt (direccion, numero);
}

int leerEnteroEnMemoria (int direccion) {
    return EEPROM.readInt(direccion + memoriaBase);
}

void escribirLongEnMemoria (int direccion, long numero) {
    EEPROM.updateLong (direccion, numero);
}

long leerLongEnMemoria (int direccion) {
    return EEPROM.readLong(direccion + memoriaBase);
}
  

void escribirFloatEnMemoria (int direccion, float numero){
    EEPROM.updateFloat(direccion, numero);
}

float leerFloatEnMemoria (int direccion) {
    return EEPROM.readFloat(direccion + memoriaBase);
}

void colocarEnBufferDatosSerial(void) {
    byte caracter;
    int aux = 0;
    int indiceAuxiliar= 0;

    while (Serial.available()) {
        caracter = Serial.read();
        aux = bufferIndiceMaximo >> 1;
        if (bufferIndice < bufferIndiceMaximo - 5) {
            bufferLectura [bufferIndice] = caracter; 
        } else {
            bufferLectura [bufferIndice] = caracter;      
            for (int i = aux; i < bufferIndiceMaximo; i++) {
                bufferIndice = i-aux;
                bufferLectura[bufferIndice] = bufferLectura[i];            
             }
        }
    bufferIndice++;
    }

}


void colocarEnBufferDatosSerial1(void) {
    byte caracter;
    int aux = 0;
    int indiceAuxiliar= 0;
    
    if (!RC[34]) {
    RC[37] = 1;
    while (Serial1.available()) {
        caracter = Serial1.read();
        aux = bufferIndiceMaximo >> 1;
        if (bufferIndice1 < bufferIndiceMaximo - 5) {
            bufferLectura1 [bufferIndice1] = caracter; 
        } else {
            bufferLectura1 [bufferIndice1] = caracter;      
            for (int i = aux; i < bufferIndiceMaximo; i++) {
                bufferIndice1 = i-aux;
                bufferLectura1[bufferIndice1] = bufferLectura1[i];            
             }
        }
    bufferIndice1++;
    }
    RC[37] = 0;
    
    }
}



void colocarEnBufferDatosSerial2(void) {
    byte caracter;
    int aux = 0;
    int indiceAuxiliar= 0;
   
    while (Serial2.available()) {
        caracter = Serial2.read();
        aux = bufferIndiceMaximo >> 1;
        if (bufferIndice2 < bufferIndiceMaximo - 5) {
            bufferLectura2 [bufferIndice2] = caracter; 
        } else {
            bufferLectura2 [bufferIndice2] = caracter;      
            for (int i = aux; i < bufferIndiceMaximo; i++) {
                bufferIndice2 = i-aux;
                bufferLectura2[bufferIndice2] = bufferLectura2[i];            
             }
        }
    bufferIndice2++;
    }
}


void colocarEnBufferDatosSerial3(void) {
    byte caracter;
    int aux = 0;
    int indiceAuxiliar= 0;

    
    while (Serial3.available()) {
        caracter = Serial3.read();
        aux = bufferIndiceMaximo >> 1;
        if (bufferIndice3 < bufferIndiceMaximo - 5) {
            bufferLectura3 [bufferIndice3] = caracter; 
        } else {
            bufferLectura3 [bufferIndice3] = caracter;      
            for (int i = aux; i < bufferIndiceMaximo; i++) {
                bufferIndice3 = i-aux;
                bufferLectura3[bufferIndice3] = bufferLectura3[i];            
             }
        }
    bufferIndice3++;
    }
}



void leerInstruccionesDeBufferSerial(byte* ptrBufferLectura, int* ptrBufferIndice, 
          byte* ptrBufferUltimaInstruccionByte, int* ptrTamanioBufferUltimaInstruccion, int numPuerto) {
   

    int encontrado = -1;
    int resultado = 0;
    int indiceAct = 0;
  
    int i = 0;
    int k = 0;

    int* tamanio = 0;
    byte* cadena;
    
    while (i < (*ptrBufferIndice - 1)) {
        
        if ((ptrBufferLectura [i] == caracterDeInicio2) && (ptrBufferLectura [i + 1] == caracterDeInicio)  ) {

            encontrado = -1;
            for (k = i; k < *ptrBufferIndice; k++) {
                if (ptrBufferLectura[k] == (byte)caracterDeFin) {
                    if (ptrBufferLectura[k + 1] == caracterDeFin2) {
                        encontrado = k + 1;
                        break;                                    
                    }
                }
            }
            
            if (encontrado < 0 ) {
                *ptrBufferIndice = 0;
                i = *ptrBufferIndice;
            } else {
              
                *ptrTamanioBufferUltimaInstruccion = 0;
                
               for (int j = i;  j < encontrado + 1 ; j++) {
                    ptrBufferUltimaInstruccionByte[*ptrTamanioBufferUltimaInstruccion] = ptrBufferLectura[j];
                    (*ptrTamanioBufferUltimaInstruccion)++;
                }
            
                i = encontrado;


                  resultado = validarInformacionTrama(numPuerto);
            }
        }
        i++;
    }
    *ptrBufferIndice = 0;
}


void imprimirCadena (int longitud, byte* Trama, int numPuerto) {

    Serial.print("\n El tamaño es ");
    Serial.print(longitud);
    for (int i = 0; i < longitud; i++) {
        Serial.print("\n");
        Serial.print (i);
        Serial.print("  ");
        Serial.print( (char)Trama[i]);
        Serial.print("  ");
        Serial.print( Trama[i],DEC);
        Serial.print("  ");
        Serial.print( Trama[i],HEX);
    }

  
}


void limpiarBuffer (int longitud, byte* Trama, int numPuerto) {

    Serial.print("\n");
    for (int i = 0; i < longitud; i++) {
        Trama[i] = 0;
        
    }

    Serial.print("\nBuffer Limpio");
  
}


int validarInformacionTrama (int longitud, byte* Trama, int numPuerto) {
  
    int longitudTrama = 0;

    int inst = 0;
    int resultado = 0;

    byte mascaraTrama1 = 0;
    byte mascaraTrama2 = 0;
    byte textoRecibidoMascara_1 = 0;
    byte textoRecibidoMascara_2 = 0;

    if (longitud >= 22) {

        inst = obtenerIntDeArregloByte(Trama + 14);


        for (int k = 0; k < 16; k++) {
            textoRecibidoMascara_1 ^= (byte) Trama[k];
            }
            mascaraTrama1 = (byte) Trama[16];

        for (int k = 20; k < longitud - 2; k++) {
            textoRecibidoMascara_2 ^= (byte) Trama[k];
            }
            mascaraTrama2 = (byte) Trama[17];
            
        longitudTrama = obtenerIntDeArregloByte(Trama + 18);
           
        if ((longitudTrama == longitud) && (textoRecibidoMascara_1 == mascaraTrama1) &&  (textoRecibidoMascara_2 == mascaraTrama2)) {
          ;

        } 
      
    }


    return resultado;
}


int validarInformacionTrama (int numPuerto) {
  
    int longitudTrama = 0;

    int inst = 0;
    int resultado = 0;

    byte mascaraTrama1 = 0;
    byte mascaraTrama2 = 0;
    byte textoRecibidoMascara_1 = 0;
    byte textoRecibidoMascara_2 = 0;

    int* tamanio = 0;
    byte* cadena;
 

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }


    if (*tamanio >= 22) {

        inst = obtenerIntDeArregloByte(cadena + 14);


        for (int k = 0; k < 16; k++) {
            textoRecibidoMascara_1 ^= (byte) cadena[k];
            }
            mascaraTrama1 = (byte) cadena[16];

        for (int k = 20; k < *tamanio - 2; k++) {
            textoRecibidoMascara_2 ^= (byte) cadena[k];
            }
            mascaraTrama2 = (byte) cadena[17];
            
        longitudTrama = obtenerIntDeArregloByte(cadena + 18);

        if ((longitudTrama == *tamanio) && (textoRecibidoMascara_1 == mascaraTrama1) &&  (textoRecibidoMascara_2 == mascaraTrama2)) {  // se descarta la información si tiene diferente longitud
            resultado = obtenerListaDeInstrucciones (numPuerto);
        } 
      
    }


    return resultado;
}


int obtenerListaDeInstrucciones (int numPuerto) {
    int inst = 0;
    int* tamanio = 0;
    byte* cadena;
    int salida = 0;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }

    inst = obtenerIntDeArregloByte(cadena + 14);

    switch (inst) {

        case INSTRUCCION_CONF_FECHAC:
            establecerFechaByte (numPuerto);
            break;
      
        case INSTRUCCION_CONF_DIGITAL:
            establecerBanderaBinaria (numPuerto, 0);
            break;

        case INSTRUCCION_CONF_ANALOG:
            modificarConfiguracionCanalAnalogico (numPuerto);
            break;

        case INSTRUCCION_CONF_ANALOG_T:
            enviarTodosDatosCanalAnalogico (numPuerto);
            break;

        case INSTRUCCION_CONF_TEMPO:
            configurarTemporizador (numPuerto);
            break;

        case INSTRUCCION_CONF_CONTA:
            configurarContador (numPuerto);
            break;
            
        case INSTRUCCION_CONF_FABRICA:
            configurarValoresDeFabrica (numPuerto);
            break;

        case INSTRUCCION_CONF_PID:
            configurarPID (numPuerto);
            break;
    }

       return salida;             
}            


void leerDatosSerial() {
    byte seleccion;
    byte caracter;
    while (Serial.available()) {  
        caracter = Serial.read();
        seleccion = leerSerial (caracter);
    }
}


int leerSerial (byte caracter) {
    int aux;
    int indiceAuxiliar= 0;
    int encontrado = -1;
    int resultado = 0;

    aux = bufferIndiceMaximo >> 1;
    if (bufferIndice < bufferIndiceMaximo - 1) {
        bufferLectura [bufferIndice] = caracter; 
    } else {
        bufferLectura [bufferIndice] = caracter;      
        for (int i = aux; i < bufferIndiceMaximo; i++) {
            bufferIndice = i-aux;
            bufferLectura[bufferIndice] = bufferLectura[i];            
         }
    }
    
    if ((bufferLectura [bufferIndice] == caracterDeFin2) && (bufferLectura [bufferIndice - 1] == caracterDeFin)  ) {
      
        int k = bufferIndice;
        encontrado = -1;
      
        for (; k > 0; k--) {
            if (bufferLectura[k] == (byte)caracterDeInicio) {
                if ( bufferLectura[k - 1] == caracterDeFin2) {
                    encontrado = k - 1 ;
                    break;                                    
                }
            }
        }
      
        if (encontrado < 0 ) {
            bufferIndice = 0;
        } else {
            tamanioBufferUltimaInstruccion = 0;
            for (int k = encontrado;  k < bufferIndice + 1; k++) {
                bufferUltimaInstruccionByte[tamanioBufferUltimaInstruccion] = bufferLectura[k];
                tamanioBufferUltimaInstruccion++;
            }
            
            for (int l = 0; l < tamanioBufferUltimaInstruccion; l++) {
                Serial.write(bufferUltimaInstruccionByte[l]);
            }

            bufferIndice = 0;
        }
    } else {
        resultado = 0;
    }
    bufferIndice++;    
    return resultado;
}

void configurarValoresDeFabrica (int numPuerto) {

    byte cantidadDeValores;
    byte direccion;
    long valor;
    
    int* tamanio = 0;
    byte* cadena;

    switch (numPuerto) {
        case PUERTO_SERIAL_0: 
            tamanio = &tamanioBufferUltimaInstruccion;
            cadena = bufferUltimaInstruccionByte;
            break;
            
        case PUERTO_SERIAL_1: 
            tamanio = &tamanioBufferUltimaInstruccion1;
            cadena = bufferUltimaInstruccionByte1;
            break;
          
      case PUERTO_SERIAL_2: 
            tamanio = &tamanioBufferUltimaInstruccion2;
            cadena = bufferUltimaInstruccionByte2;
            break;
            
      case PUERTO_SERIAL_3: 
            tamanio = &tamanioBufferUltimaInstruccion3;
            cadena = bufferUltimaInstruccionByte3;        
            break;
    }


    imprimirInicioTexto2(INSTRUCCION_RECIBIDA_CONF_FABRICA );
    imprimirFinalTexto2(numPuerto);
    
    direccion = obtenerIntDeArregloByte(cadena + 20);
    valor = (int) obtenerByteDeArregloByte(cadena  + 22);

    
        Serial.print("\n");
        Serial.print("Z [");
        Serial.print(direccion);
        Serial.print ("]= ");
        Serial.print (valor);
        Serial.print("\n");

}



void establecerConfiguracion (int numPuerto, int opcion) {
    byte numeroByte  = 0;
    int numeroInt = 0;
    float numeroFloat = 0;
    long numeroLong = 0;

    switch (opcion) { 

        case EEPROM_TEMPORIZADORES:
            break;
            
        case EEPROM_HORA:

            numeroByte = obtenerByteDeArregloByte (bufferUltimaInstruccionByte + 9);
            
            if ((numeroByte > 15 ) | (numeroByte < 0)) {
                numeroByte = 0;
            }
            
            numeroLong = obtenerLongDeArregloByte (bufferUltimaInstruccionByte + 9 + 1);
            escribirLongEnMemoria ((numeroByte) * sizeof(long) + obtenerDireccionMemoriaEEPROM (EEPROM_HORA), numeroLong);
            obtenerConfiguracion (0, EEPROM_HORA);
            break;
      
        }

}

void obtenerConfiguracion (int numPuerto, int opcion) {
    byte bandera;
  
    switch (opcion) { 

        case EEPROM_SENSORES:
            for (int i = 0; i < numeroDeSensores; i++) {
                leerVariablesAnalogicasDeEEPROM (i);
            }
            imprimirInicioTexto2(INSTRUCCION_LEIDO_DE_EEPROM_ANALOG);
            escribirNumeroIntEnTexto(obtenerDireccionMemoriaEEPROM (EEPROM_SENSORES));
            imprimirFinalTexto2(numPuerto);
            break;
      
        case EEPROM_TEMPORIZADORES:
            for (int i = 0; i < numeroDeTemporizadores; i++) {
                leerTemporizadoresDeEEPROM(i);
            }
            
            imprimirInicioTexto2(INSTRUCCION_LEIDO_DE_EEPROM_TEMPO);
            escribirNumeroIntEnTexto(obtenerDireccionMemoriaEEPROM (EEPROM_TEMPORIZADORES));
            imprimirFinalTexto2(numPuerto);
            break;
            
        case EEPROM_CONTADORES:
            for (int i = 0; i < numeroDeContadores; i++) {
                leerContadoresDeEEPROM(i);
            }
            imprimirInicioTexto2(INSTRUCCION_LEIDO_DE_EEPROM_CONTA);
            escribirNumeroIntEnTexto(obtenerDireccionMemoriaEEPROM (EEPROM_CONTADORES));
            imprimirFinalTexto2(numPuerto);
            break;
            
        case EEPROM_PID:
            for (int i = 0; i < numeroDePID; i++) {
                leerPIDDeEEPROM(i);
            }
            imprimirInicioTexto2(INSTRUCCION_LEIDO_DE_EEPROM_PID);
            escribirNumeroIntEnTexto(obtenerDireccionMemoriaEEPROM (EEPROM_PID));
            imprimirFinalTexto2(numPuerto);
            break;
            
        case EEPROM_ALARMAS:

            for (int i = 0; i < numeroDeAlarmas; i++) {

                leerAlarmasDeEEPROM(i);
            }
            imprimirInicioTexto2(INSTRUCCION_LEIDO_DE_EEPROM_ALARMA);
            escribirNumeroIntEnTexto(obtenerDireccionMemoriaEEPROM (EEPROM_ALARMAS));
            imprimirFinalTexto2(numPuerto);  
            break;
      
        case EEPROM_HORA:

            for (int i = 0; i < numeroDeDias; i++) {

            }
            
            imprimirInicioTexto2(INSTRUCCION_LEIDO_DE_EEPROM_HORARIO);
            escribirNumeroIntEnTexto(obtenerDireccionMemoriaEEPROM (EEPROM_HORA));
            imprimirFinalTexto2(numPuerto);
            break;    
      
     }
  }

void imprimirVersion (int numPuerto) {

    imprimirInicioTexto2(INSTRUCCION_MOSTRAR_VERSION);
    
    escribirStringEnTexto(F("Version "));
    escribirStringEnTexto(F("0.9.5 "));
    escribirStringEnTexto(F("\n arduino MEGA 2560 "));
    escribirStringEnTexto(F("\n Sigfrido Soria Frias "));    
    
    imprimirFinalTexto2(numPuerto);
}


void enviarInformacion (int numPuerto) {
    imprimirFechaByte (numPuerto);
    imprimirDatosAnalogicos (numPuerto);
    imprimirDatosDigitales (numPuerto);
    imprimirTemporizadores (numPuerto);
    imprimirContadores (numPuerto);
//    imprimirHorario (numPuerto);
    imprimirAlarmas (numPuerto);
    imprimirValoresPID (numPuerto);    
}

void verificarComunicacion (int numPuerto) {
    imprimirInicioTexto2(INSTRUCCION_RECIBIDO);
    imprimirFinalTexto2(numPuerto);
}


float calcularPendiente (float x0, float y0, float x1, float y1) {
    if ( x0 != x1) {
      return (float)(y0 - y1)/(x0 - x1);      
    } else {
        return 0;
    }
}

float calcularOrdenada (float x0, float y0, float m) {
    return (y0 - m*x0);
}

float calcularOrdenada (float x0, float y0, float x1, float y1) {
    float m = 0; 
    m = calcularPendiente (x0, y0, x1, y1);
    return calcularOrdenada (x0, y0, m);
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
  

float obtenerFloatDeString (String cadena) {
    float numero = 0;
    int indiceDelPunto = -1;
    String num;
    int aux0;
    int aux1;
    
    indiceDelPunto = cadena.indexOf('.');
    aux1 = cadena.length();

    if (indiceDelPunto != -1)
        {
        num = cadena.substring(indiceDelPunto + 1, aux1 );
        aux0 = num.length();
        numero =((float) num.toInt());
        for (int i = 0; i < aux0; i++) {
            numero /=10; 
        } 
    }else {
      indiceDelPunto = aux1;
    }
    
    
    if (cadena[0] == '-') {
        num = cadena.substring(1, indiceDelPunto);
        numero += (float)num.toInt();
        numero = -numero;
    } else {
        num = cadena.substring(0, indiceDelPunto);
        numero += (float)num.toInt();
    }
    return numero;
}

int obtenerIntDeArregloByte (byte* arreglo) {
    int *punteroInt;
    punteroInt = (int*) arreglo;
    return *punteroInt;  
  }
  

byte obtenerByteDeArregloByte (byte* arreglo) {
    byte *punteroByte;
    punteroByte = (byte*) arreglo;
    return *punteroByte;  
}  
  

void obtenerBitsDeArregloByte (byte *arreglo, int *arregloEntero, int numeroDeBytesADescomponer, int numPuerto)  {
    int numeroEntero;
    int indiceInicial = 0;
    byte numeroByte;

    imprimirInicioTexto2(INSTRUCCION_MOSTRAR_BITS);

    for (int j = 0; j < numeroDeBytesADescomponer; j++) {
        numeroEntero = 0;
        numeroByte = (byte) arreglo [j + indiceInicial];

        for (int i = 0; i < 8; i++){
            numeroEntero = (numeroByte >> i) & 1;
            
            arregloEntero [ i + 8*j] = numeroEntero;

            escribirNumeroIntEnTexto(numeroEntero); 
        }
    }
    imprimirFinalTexto2(numPuerto);
}
  
  
  
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}


String obtenerCadena (byte arreglo[], int inicio, int fin) {
    String cadena = "";
    
    for (int k = inicio; k < fin; k++) {
        cadena += (char)arreglo[k];
    }
    return cadena;
}


void bufferTextoLimpiar (void) {
    for (int i = 0; i < bufferTextoMaximo; i++) {
        bufferTexto[i] = (byte) 0;
    }
    bufferTextoIndice = 0;
    textoLongitud = 0;
    textoMascara = 0;
}



void imprimirInicioTexto2(int instruccion) {
    bufferTextoLimpiar();
    textoSecuencia++;
    
    escribirCaracterEnTexto (caracterDeInicio2);
    escribirCaracterEnTexto (caracterDeInicio);
    escribirStringEnTexto (textoOrigen);

    escribirNumeroLongEnTexto (textoSecuencia);
    escribirNumeroLongEnTexto (textoDestino);  

    escribirNumeroIntEnTexto (instruccion);
    escribirByteEnTexto(textoMascara);
    textoMascara = 0;
    bufferTextoIndice +=3;

}


void imprimirFinalTexto2(int numPuerto) {
    int indiceAux = bufferTextoIndice;

    bufferTextoIndice = 17;
    escribirByteEnTexto(textoMascara);
    escribirNumeroIntEnTexto(textoLongitud + 5);
    
    bufferTextoIndice = indiceAux;
 
    bufferTexto[bufferTextoIndice++] = '\n';

    escribirCaracterEnTexto (caracterDeFin);
    escribirCaracterEnTexto (caracterDeFin2);
    
    reImprimirEnPuerto(numPuerto);
}


void reImprimirEnPuerto(int numPuerto) {
  switch (numPuerto) {
    case PUERTO_SERIAL_0:     // 
      Serial.write(bufferTexto, bufferTextoIndice);
    break;

    case PUERTO_SERIAL_1:     // 
      Serial1.write(bufferTexto, bufferTextoIndice);
    break;
    
    case PUERTO_SERIAL_2:     // 
      Serial2.write(bufferTexto, bufferTextoIndice);
    break;

    case PUERTO_SERIAL_3:     // 
     Serial3.write(bufferTexto, bufferTextoIndice);
    break;
    }   
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
	textoMascara ^=(byte) cadena[i];
    }
} 
