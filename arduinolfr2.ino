//Code snipped of workign analog lfr
//Array codes
//Theshold finder
int pins[7] = { A0, A1, A2, A3, A4, A5,A6 };
int sensorArray[7];
int sensorValue = 0;
double totalSum = 0, weightedSum = 0;
double distFromCenter[7] = { 0,-30, -20, -10, 10, 20, 30 };
int threshold=180;
int maxi[7] = { 793, 830, 684, 793, 660, 788, 820 };
int mini[7] = { 85, 79, 66, 99, 69, 87, 100 };
int L1 = 2;
int L2= 6;
int R1= 4;
int R2 = 7;
int enA = 3; //left
int enB = 5;  //rigt


int max_speed1 = 130;  // blue
int max_speed2 = 100;

//pid
double Kp = 10;    // 3.75
double Kd = 0;  // 50
double Ki = 0;
double P = 0;
double I = 0;
double D = 0;
double lastError = 0;
double lastI = 0;
double PID = 0;
int error;

char path[50];
int path_length;

void readArray() {
  for (int i = 0; i <7; i++) {
    sensorArray[i] = constrain(map(analogRead(pins[i]), mini[i], maxi[i], 0, 255), 0,255);
  }

  sensorValue = 0;
  totalSum = 0;
  weightedSum = 0;
  for (int i = 0; i <= 6; i++) {
    totalSum += sensorArray[i];
    weightedSum += distFromCenter[i] * sensorArray[i];
    if (sensorArray[i] > threshold) sensorValue |= 1 << (6-i);
  }
  //Serial.print("sensor value: ");
  //Serial.print(sensorValue);
  error = weightedSum / totalSum;
}
void calculatePID() {
  P = error;
  I = I + lastI;
  D = error - lastError;
  PID = P * Kp + I * Ki + D * Kd;

  lastI = I;
  lastError = error;
  // Set wheel movement based on errorValue
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(R1,0);
  digitalWrite(R2,0);
  digitalWrite(L1,0);
  digitalWrite(L2,0);
  delay(100);
}
void followline() {
  // moveForward();
  while(1){
    error = 0;
    lastError = 0;


    readArray();
    // Serial.println(String(sensorValue) + " " + String(PID));
    calculatePID();

    // switch +, - on white lines
    digitalWrite(R1,1);
    digitalWrite(R2,0);
    digitalWrite(L1,1);
    digitalWrite(L2,0);
    analogWrite(enB, constrain(max_speed1 - PID, 0, 255));
    analogWrite(enA, constrain(max_speed2 + PID, 0, 255));
    if (sensorValue == 0b000000 || sensorValue == 0b1111111) {
      return;
    } 
    else if (sensorValue == 0b1111100 || sensorValue == 0b1111110 || sensorValue == 0b1001111 || sensorValue == 0b1011111) {
      return;
    }
    else if (sensorValue == 0b0111100 || sensorValue == 0b0111110 || sensorValue == 0b0001111 || sensorValue == 0b0011111) {
      return;
    }
  }
    delay(4);
}
void leftTurn(int d) {
  // do {
  analogWrite(enA, 250);
  analogWrite(enB, 250);
  digitalWrite(R1,1);
  digitalWrite(R2,0);
  digitalWrite(L1,0);
  digitalWrite(L2,1);
  delay(d);
}

void rightTurn(int d) {
  // do {
  analogWrite(enA, max_speed1);
  analogWrite(enB, max_speed2);
  digitalWrite(R1,0);
  digitalWrite(R2,1);
  digitalWrite(L1,1);
  digitalWrite(L2,0);
  delay(d);
}
void UTurn(int d) {
  // do {
  analogWrite(enA, max_speed1);
  analogWrite(enB, max_speed2);
  while (sensorArray[3] < threshold && sensorArray[4] < threshold) {
  digitalWrite(R1,1);
  digitalWrite(R2,0);
  digitalWrite(L1,0);
  digitalWrite(L2,1);
  delay(d);
  
  readArray();
  }
}

void rightfirst() {
  
  // Stop on all white or all black

  
   
  if (sensorArray[6] > threshold) {
    Serial.println("Right turn");
    rightTurn(225);
    path[path_length++] = 'R';

                                                                            
    
  }
  else if (sensorArray[3] > threshold || sensorArray[4] > threshold) {
    Serial.println("Straight");
    path[path_length++] = 'S';
    
  } 
  
  else if ( sensorArray[1] > threshold) {
    Serial.println("Left turn");
    stop();
    leftTurn(200);
    path[path_length++] = 'L';
  
  }
  
  /*else if (sensorValue == 0b0000000) {
    Serial.println("U turn");
    UTurn(10);
    path[path_length++] = 'B';
    
  }*/
  
}



 /* while (sensorArray[3] > threshold || sensorArray[4]>threshold) {
    digitalWrite(L1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(100);

    readArray();
  }

  while (sensorArray[3] < threshold && sensorArray[4] < threshold) {
    digitalWrite(motInp1, HIGH);
    digitalWrite(motInp2, LOW);
    digitalWrite(motInp3, HIGH);
    digitalWrite(motInp4, LOW);
    delay(2);
    readArray();
  }*/
  





void setup(){
Serial.begin(9600);
}
void loop(){
  readArray();
    delay(10);
    if (sensorValue != 0 && sensorValue != 0b111111) {
      followline();
      Serial.println("followline");
  }
  stop();
  Serial.println("stopping");
  if (sensorValue != 0 && sensorValue != 0b1111111) {
    rightfirst();
  }

}