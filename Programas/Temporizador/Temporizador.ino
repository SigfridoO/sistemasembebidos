
#define numeroDeX 16
byte X[numeroDeX];


#define numeroDeY 16
byte Y[numeroDeY];


int DI_00 = 35;
int DI_01 = 34;
int DI_02 = 39;
int DI_03 = 36;


int DO_00 = 12;
int DO_01 = 14;
int DO_02 = 27;
int DO_03 = 26;
int DO_04 = 13;
int DO_05 = 2;

void leerPines (void);


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
}


void loop() {

  leerPines();


  Y[0] = X[0];
  Y[1] = X[1];
  Y[2] = X[2];
  Y[3] = X[3];
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
