
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
  digitalWrite(DO_0, HIGH);
  digitalWrite(DO_1, HIGH);
  digitalWrite(DO_2, HIGH);
  digitalWrite(DO_3, HIGH);
  digitalWrite(DO_4, HIGH);
  digitalWrite(DO_5, HIGH);


  
  delay(1000);             
  digitalWrite(DO_0, LOW);
  digitalWrite(DO_1, LOW);
  digitalWrite(DO_2, LOW);
  digitalWrite(DO_3, LOW);
  digitalWrite(DO_4, LOW);
  digitalWrite(DO_5, LOW);
  delay(1000);            
}
