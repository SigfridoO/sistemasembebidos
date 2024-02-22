
#define numeroDeX = 16
byte X[numeroDeX];


#define numeroDeY = 16
byte Y[numeroDeY];


int DI_0 = 36;
int DI_1 = 39;
int DI_2 = 34;
int DI_3 = 35;


int DO_0 = 12;
int DO_1 = 14;
int DO_2 = 27;
int DO_3 = 26;
int DO_4 = 13;
int DO_5 = 2;

void leerPines (void);


void setup() {

  pinMode(DI_0, INPUT);
  pinMode(DI_1, INPUT);
  pinMode(DI_2, INPUT);
  pinMode(DI_3, INPUT);
  
  pinMode(DO_0, OUTPUT);
  pinMode(DO_1, OUTPUT);
  pinMode(DO_2, OUTPUT);
  pinMode(DO_3, OUTPUT);
  pinMode(DO_4, OUTPUT);
  pinMode(DO_5, OUTPUT);
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
