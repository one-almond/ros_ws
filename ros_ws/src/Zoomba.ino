int ENA[] =  {4,2};
int ENB[] =  {3,1};
int SPD[] =  {6,5};
int SCALE[] = {1,1};

void setup() {
  pinMode(ENA[0], OUTPUT);
  pinMode(ENB[0], OUTPUT);
  pinMode(ENA[1], OUTPUT);
  pinMode(ENB[1], OUTPUT);
  pinMode(SPD[0], OUTPUT);
  pinMode(SPD[1], OUTPUT);

  digitalWrite(ENA[0], HIGH);
  digitalWrite(ENB[0], LOW);
  digitalWrite(ENA[1], HIGH);
  digitalWrite(ENB[1], LOW);


  Serial.begin(115200);
  Serial.println("READY");
  Serial.println("DRIVE Lxxx Rxxx");

}

void loop() {
  // put your main code here, to run repeatedly:
    handleSerial();
}

void handleSerial() {
  long left = 0;
  long right = 0;
  long SPDS[2];
  
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("DRIVE")) {
      sscanf(cmd.c_str(), "DRIVE L%ld R%ld", &left, &right);
      SPDS[0] = left;
      SPDS[1] = right;
      wheel(SPDS);
    }


}

void wheel(long SPDS[]){
  for (int i =0; i < 2; i++){
    if (abs(SPDS[i]) - 0.2 > 0){
      if (SPDS[i] > 0){
        digitalWrite(ENA[i], HIGH);
        digitalWrite(ENB[i], LOW);
      } else {
        Serial.println("backward");
        digitalWrite(ENA[i], LOW);
        digitalWrite(ENB[i], HIGH);
      } 
      analogWrite(SPD[i], SCALE[i]*abs(SPDS[i]));
    } else {
    analogWrite(SPD[i], 0);
    } 
  }

}


