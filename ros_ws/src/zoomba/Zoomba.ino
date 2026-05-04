int ENA[] =  {4,7};
int ENB[] =  {3,2};
int SPD[] =  {6,5};
int SCALE[] = {100,100};

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
  float left = 0;
  float right = 0;
  float SPDS[2];
  
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("DRIVE")) {
      left = atof((cmd.substring(cmd.indexOf('L')+1,cmd.indexOf(' ',cmd.indexOf('L')+1))).c_str());
      right = atof((cmd.substring(cmd.indexOf('R',cmd.indexOf('L'))+1,cmd.length())).c_str());
      //sscanf(cmd.c_str(), "DRIVE L%f R%f", &left, &right);
      SPDS[0] = left;
      SPDS[1] = right;
      wheel(SPDS);
      Serial.println(cmd.substring(cmd.indexOf('R')+1, cmd.length()));
      Serial.println(String(left)+String(right));
    }


}

void wheel(float SPDS[]){
  for (int i =0; i < 2; i++){
    if (abs(SPDS[i]) - 0.2 > 0){
      if (SPDS[i] > 0){
        digitalWrite(ENA[i], HIGH);
        digitalWrite(ENB[i], LOW);
        Serial.println("ENA" + String(i) + " pin:" + String(ENA[i]) + "=HIGH");
        Serial.println("ENB" + String(i) + " pin:" + String(ENB[i]) + "=LOW");
      } else {
        digitalWrite(ENA[i], LOW);
        digitalWrite(ENB[i], HIGH);
        Serial.println("ENA" + String(i) + " pin:" + String(ENA[i]) + "=LOW");
        Serial.println("ENB" + String(i) + " pin:" + String(ENB[i]) + "=HIGH");
      } 
      analogWrite(SPD[i], SCALE[i]*abs(SPDS[i]));
    } else {
    analogWrite(SPD[i], 0);
    } 
  }

}
