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


int contador;

void setup() {
  Serial.begin(9600);
  TON[0].tiempo = (unsigned long)1 * 1000;      // 1 segundo

  contador = 0;
}

void loop() {
  TON[0].entrada = !TON[0].salida;
  actualizarTON(0);

  if (TON[0].salida) {
    contador += 1;
  }

  //Serial.print("contador: ");
  //Serial.write(contador);
  //Serial.print("\n");

  //Serial.print ("TON0");
  //Serial.print("\t");
  //Serial.print(TON[0].tiempoActual);
  //Serial.print("\t");
  //Serial.print(TON[0].salida);
  //Serial.print("\n");

   while (Serial.available()) {
    int caracter = Serial.read();
    Serial.write(caracter*10);
    
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
