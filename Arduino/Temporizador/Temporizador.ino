
#define numeroDeX 16
byte X[numeroDeX];


#define numeroDeY 16
byte Y[numeroDeY];





int DI_00 = 36;
int DI_01 = 39;
int DI_02 = 34;
int DI_03 = 35;





int DO_00 = 26;
int DO_01 = 27;
int DO_02 = 14;
int DO_03 = 12;
int DO_04 = 13;

int DO_05 = 2;

void leerPines (void);




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

  Serial.begin(9600);


}


void loop() {

  leerPines();


  Y[0] = X[0];
  Y[1] = X[1];
  Y[2] = X[2];
  



  TON[0].entrada = !TON[1].salida;
  actualizarTON(0);

  TON[1].entrada = TON[0].salida;
  actualizarTON(1);

  Y[4] = !TON[1].salida & TON[0].salida; 





  TON[2].entrada = X[3] & !TON[3].salida;
  actualizarTON(2);

  TON[3].entrada = TON[2].salida;
  actualizarTON(3);

  Y[3] = X[3] & !TON[3].salida & !TON[2].salida; 

  
  Serial.print ("\n");
  Serial.print (TON[0].salida);
  Serial.print ("\t");
  Serial.print (TON[0].tiempoActual);
    
  Serial.print ("\t");
  Serial.print (Y[4]);
  


  
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

  TON[0].tiempo = (unsigned long)1 * 3000;
  TON[1].tiempo = (unsigned long)2 * 2000;

  TON[2].tiempo = (unsigned long)1 * 3000;
  TON[3].tiempo = (unsigned long)1 * 3000;
  TON[4].tiempo = (unsigned long)1 * 80;
}
