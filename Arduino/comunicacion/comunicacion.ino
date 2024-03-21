

// ----------  Señales digitales
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
int DO_05 = 2;    // LED CONECTADO a la tarjeta

// ----------  Temporizadores
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

// ----------  Comunicación
/*
    00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20
   |CI| I  N V | TI  | NUM | LON | DATOS ...........        |V |CF| 
    CI  - Caracter de inicio
    INV - Para invernadero
    TI  - Tipo de instrucción
    NUM - Numero de instrucción
    LON - Longitud
    V   - Verificación
    CF  - Caracter de final
 */
#define bufferIndiceMaximo  120
byte bufferLectura[bufferIndiceMaximo];
int bufferIndice = 0;

void colocarDatosEnBufferSerial(void);
void imprimirTrama(byte *, int, int);
 
void leerInstruccion(int);




int contador;

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

  Serial.begin(9600);
  
  TON[0].tiempo = (unsigned long)1 * 1000;     // 1 segundo
  TON[1].tiempo = (unsigned long)1 * 400;      // 0.4 segundos
  TON[2].tiempo = (unsigned long)1 * 800;      // 0.8 segundos

  
  contador = 0;
}

void loop() {
  colocarDatosEnBufferSerial();
  
  TON[0].entrada = !TON[0].salida;
  actualizarTON(0);

  if (TON[0].salida) {
    contador += 1;
  }

  // Parpadeo indicando que el programa esta funcionando
  TON[1].entrada = !TON[2].salida;
  actualizarTON(1);

  TON[2].entrada = TON[1].salida;
  actualizarTON(2);

  Y[5] = TON[1].salida & !TON[2].salida;

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

  //Serial.print("contador: ");
  //Serial.write(contador);
  //Serial.print("\n");

  //Serial.print ("TON0");
  //Serial.print("\t");
  //Serial.print(TON[0].tiempoActual);
  //Serial.print("\t");
  //Serial.print(TON[0].salida);
  //Serial.print("\n");



}
// ----------  Comunicación

void colocarDatosEnBufferSerial(){
  byte caracter;
  int aux = 0;
  int indiceAuxiliar = 0;

  while (Serial.available()) {
    caracter = Serial.read();
    
    bufferLectura[bufferIndice++] = caracter;
    

    if (bufferIndice > bufferIndiceMaximo - 10) {
      aux = bufferIndiceMaximo >> 1;
      for (int i = aux; i < bufferIndiceMaximo; i++){
        bufferIndice = i - aux;
        bufferLectura[bufferIndice] = bufferLectura[i];
      }
    }

    imprimirTrama(bufferLectura, 0, bufferIndice);
    
   }
}
void imprimirTrama(byte *ptrTrama, int inicio, int tamanio) {
  Serial.print("\n>>");
  for (int k = inicio; k < inicio + tamanio; k++) {
    Serial.write(ptrTrama[k]);
  }
  
}


void leerInstruccion(int caracter) {
      switch (caracter) {
        case 1:
          Serial.print("Instruccion 1");
          break;

        case 2:
          Serial.print("Instruccion 2");
          break;

        case 3:
          Serial.print("Instruccion 3");
          break;

         
          
    }
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
