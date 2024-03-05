

// ----------  Entradas salidas
int DO_00 = 26;


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
void leerInstruccion(int);




int contador;

void setup() {

  pinMode(DO_00, OUTPUT);
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
    //Serial.write(caracter*10);
    leerInstruccion(caracter);
      
    
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
