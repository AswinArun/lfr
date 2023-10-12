int R1=10;
int R2=9;
int enA=6;//right
int L1=3;
int L2=4;
int enB=5;//left
int pins[7] = { A0, A1, A2, A3, A4, A5,A6 };

int maxi[7] = {972, 957, 967, 898, 953, 971, 958};
int mini[7] = {138, 201, 146, 119, 79, 119, 274};
double distFromCenter[7] = { 0, -20, -13, -5, 5, 13, 20 };
int threshold=180;
//pid
char path[100];
int path_length = 0;
int node = 0;
double Kp =18;  // 15
double Kd = 150;  // 25
double Ki = 0.005;
double P = 0;
double I = 0;
double D = 0;
double lastError = 0;
double lastI = 0;
double PID = 0;
int error;


int LED = 2;
int algoSwitch = 11;
int finalSwitch = 7;

int algoFlag = 0; //0 for lsrb, 1 for rslb
int finalFlag = 0;

int max_speed1 = 200;  //right enb
int max_speed2 = 200;
int min_speed1 = 100;
int min_speed2 = 100;
int sensorArray[7];
int totalSum;
int weightedSum;
int sensorValue;

int left = 0, right = 0; //flag for extreme left/right

void pathSimplify()
{

  int B=1;

  while(B)
  {
    B = 0;
    int temp = 0; //variable to iterate through path
    int k=0;  //index to store values
    int minus=0; //to store size decrement in path_length
    for (temp; temp < path_length-2; temp++)
    {
      if (path[temp + 1] != 'B'){
        path[k++] = path[temp];
        continue;
      }
      int total = 0;
      int m;
      for (m=0; m <= 2; m++){
        switch(path[temp+m]){
          case 'R':
            total += 1;
            break;
          case 'S':
            break;
          case 'L':
            total += 3;
            break;
          case 'B':
            total += 2;
            break;
        }
      }
      total = total % 4;
      switch(total){
        case 0:
          path[k++] = 'S';
          break;
        case 1:
          path[k++] = 'R';
          break;
        case 2:
          path[k++] = 'B';
          B=1; //setting flag to run simplification again
          break;
        case 3:
          path[k++] = 'L';
          break;
      }
      minus += 2;
      temp+=2;
    }
    path[k++] = path[temp++];
    path[k++] = path[temp++];
    path_length -= minus;

    Serial.print("\nsimplification iteration: ");
    for (int c=0; c<path_length; c++)
    {
      Serial.print(path[c]);
      Serial.print(" ");
    }
    Serial.println();
  }

}

void stop(int d){
   digitalWrite(R1, 0);
    digitalWrite(R2, 0);
    digitalWrite(L1, 0);
    digitalWrite(L2, 0);
    analogWrite(enB, 0);
    analogWrite(enA, 0);
    delay(d);

}
void leftTurn(int d) {
  readArray();
  analogWrite(enA, 150);
  analogWrite(enB, 150);

  // while (sensorArray[2] > threshold) {
  //   digitalWrite(R1, 1);
  //   digitalWrite(R2, 0);
  //   digitalWrite(L1, 1);
  //   digitalWrite(L2, 0);
  //   readArray();
  //   delay(d);
  // }
  while (sensorArray[1] < threshold) {
    digitalWrite(R1, 1);
    digitalWrite(R2, 0);
    digitalWrite(L1, 0);
    digitalWrite(L2, 1);
    readArray();
   
  }
  while (sensorArray[1] > threshold) {
    digitalWrite(R1, 1);
    digitalWrite(R2, 0);
    digitalWrite(L1, 0);
    digitalWrite(L2, 1);
    readArray();
   
  }
  while (sensorArray[2] < threshold) {
    digitalWrite(R1, 1);
    digitalWrite(R2, 0);
    digitalWrite(L1, 0);
    digitalWrite(L2, 1);
    readArray();
   
  }

  // while(sensorArray[3]<threshold){
  //   digitalWrite(R1, 1);
  //   digitalWrite(R2, 0);
  //   digitalWrite(L1, 0);
  //   digitalWrite(L2, 1);
  //   readArray();
   
  // }
  digitalWrite(R1, 0);
  digitalWrite(R2, 1);
  digitalWrite(L1, 1);
  digitalWrite(L2, 0);
  delay(50);

}


void rollback(int d){
   digitalWrite(R1, 0);
  digitalWrite(R2, 1);
  digitalWrite(L1, 0);
  digitalWrite(L2, 1);
  analogWrite(enA,255);
  analogWrite(enB,255);
  left = 0; right = 0;
  for(int i = 0; i<d; i++){
    readArray();
    if (sensorArray[6] > threshold)
      right = 1;
    if(sensorArray[1]>threshold)
      left = 1;
    delay(2);
  }
}
void rightTurn(int d) {
 
  readArray();
  analogWrite(enA, 140);
  analogWrite(enB, 140);
  // while (sensorArray[6] > threshold) {
  //   digitalWrite(R1, 1);
  //   digitalWrite(R2, 0);
  //   digitalWrite(L1, 1);
  //   digitalWrite(L2, 0);
  //   readArray();
  //   delay(d);
  // }
  while (sensorArray[6] > threshold) {
    digitalWrite(R1, 0);
    digitalWrite(R2, 1);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    readArray();
   
  }
 
  while (sensorArray[6] < threshold) {
    digitalWrite(R1, 0);
    digitalWrite(R2, 1);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    readArray();
   
  }
  while(sensorArray[4] < threshold){
    digitalWrite(R1, 0);
    digitalWrite(R2, 1);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    readArray();
   
  }

  // for small return
  digitalWrite(R1, 1);
  digitalWrite(R2, 0);
  digitalWrite(L1, 0);
  digitalWrite(L2, 1);
  delay(50);
}
void UTurn(int d) {
  // do {
  analogWrite(enA, 160);
  analogWrite(enB, 160);
  while (sensorArray[5] < threshold) {
    digitalWrite(R1, 0);
    digitalWrite(R2, 1);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    delay(d);

    readArray();
  }
  digitalWrite(R1, 1);
  digitalWrite(R2, 0);
  digitalWrite(L1, 0);
  digitalWrite(L2, 1);
  delay(50);
}
void moveforward(int d) {
  analogWrite(enA, 150);
  analogWrite(enB, 150);

  digitalWrite(R1, 1);
  digitalWrite(R2, 0);
  digitalWrite(L1, 1);
  digitalWrite(L2, 0);
  delay(d);
 
}

void readArray() {
  sensorValue = 0;
  totalSum = 0;
  weightedSum = 0;
  for (int i = 0; i < 7; i++) {
    sensorArray[i] = constrain(map(analogRead(pins[i]), mini[i], maxi[i], 0, 255), 0, 255);
    if (sensorArray[i] > threshold) sensorValue |= 1 << (6 - i);
  }
   
 
  for (int i = 2; i <= 5; i++) {
    totalSum += sensorArray[i];
    weightedSum += distFromCenter[i] * sensorArray[i];
   
  }
  //Serial.print("sensor value: ");
  //Serial.print(sensorValue);
  error = weightedSum / totalSum;


 
}
void calculatePID() {
  P = error;
  I = I + error;
  D = error - lastError;
  PID = P * Kp + I * Ki + D * Kd;
  lastError = error;
  // Set wheel movement based on errorValue
}

void followline() {
  error = 0;
  lastError = 0;

  while (1) {
    readArray();

    calculatePID();

    digitalWrite(R1, 1);
    digitalWrite(R2, 0);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    analogWrite(enB, constrain(max_speed1 +PID, 0, 255));
    analogWrite(enA, constrain(max_speed2 - PID, 0, 255));
   
   if (sensorArray[1]>threshold || sensorArray[6]>threshold || sensorValue == 0){
     rollback(10);
     return;
   }
    delay(5);
  //   if(sensorValue==0 || sensorValue==0b1111111){
  // delay(10);
  // stop(1000);
  // return;}

  // if(sensorValue==0b0001111 || sensorValue==0b0111100){
  // delay(10);
 
  // return;}
  // if(sensorValue==0b0000111 || sensorValue==0b0111000)
  // delay(10);
  // return;
  // if(sensorArray[1]<threshold && senosr)


  }

}

void leftfirst() {

  // Stop on all white or all black

  // Serial.println("right first called");
  int sensor1 = sensorArray[1], sensor2 = sensorArray[2], sensor3 = sensorArray[3];
  int sensor5 = sensorArray[5], sensor6 = sensorArray[6], sensor4 = sensorArray[4];
  int prevValue = sensorValue;
  // moveforward(50);
   Serial.println("sensorValue");
  Serial.print(sensorValue, BIN);
  readArray();
  if(sensorValue==0b1111110 || sensorValue==0b1111111 ||(sensorValue==0b1111100 && prevValue == sensorValue) )
  {
    moveforward(50);
    readArray();
    if(sensorValue==0b1111110 || sensorValue==0b1111111 ||(sensorValue==0b1111100 && prevValue == sensorValue) )
      end();
    else{
      path[path_length++] = 'L';
      leftTurn(10);
    }
  }

  // else if ((sensor1 > threshold || sensor2 > threshold) && sensor3 > threshold) {

  //   // Serial.println("Left turn");
  //   leftTurn(10);
  //   path[path_length++] = 'L';
  // }

  else if (left == 1) {
    // Serial.println("Left turn");
    leftTurn(10);
    path[path_length++] = 'L';
  }

  else if (sensorArray[0]>threshold || sensorArray[3] > threshold || sensorArray[4]>threshold || sensorArray[1]>threshold || sensorArray[2]>threshold) {
    // Serial.println("Straight");
    path[path_length++] = 'S';
    moveforward(100);
  }

  else if (right == 1) {

    // Serial.println("Right turn");
    rightTurn(10);
    path[path_length++] = 'R';
  }

  else if (sensorValue == 0b0000000) {
    // Serial.println("U turn");
    UTurn(10);
    path[path_length++] = 'B';
  }
}

void rightfirst() {

  // Stop on all white or all black
  // Serial.print("sensor value: ");
  // Serial.println(sensorValue, BIN);
 
  int sensor1 = sensorArray[1], sensor2=sensorArray[2];
  int sensor5 = sensorArray[5], sensor6 = sensorArray[6], sensor4 = sensorArray[4];
  int prevValue = sensorValue;
  // moveforward(50);
  readArray();
  if(sensorValue==0b1111110 || sensorValue==0b1111111 ||(sensorValue==0b1111100 && prevValue == sensorValue) )
  {
    moveforward(50);
    readArray();
    if(sensorValue==0b1111110 || sensorValue==0b1111111 ||(sensorValue==0b1111100 && prevValue == sensorValue) )
      end();
    else{
      path[path_length++] = 'S';
    }
  }
 
  //  if ((sensor6 > threshold || sensor5 > threshold) && sensor1 > threshold) {

  //   // Serial.println("Right turn");
  //   rightTurn(10);
  //   path[path_length++] = 'R';
  // }

  else if (right ==1) {
    // Serial.println("Right turn");
    rightTurn(10);
    path[path_length++] = 'R';
  }

  else if (sensorArray[0]>threshold || sensorArray[3]>threshold || sensorArray[4]>threshold || sensorArray[5]>threshold || sensorArray[6]>threshold) {
    // Serial.println("Straight");
    path[path_length++] = 'S';
    moveforward(100);
  }

  else if (left == 1 ) {
    // Serial.println("Left turn");
    leftTurn(10);
    path[path_length++] = 'L';
  }

  else if (sensorValue == 0b0000000) {
    // Serial.println("U turn");
    UTurn(10);
    path[path_length++] = 'B';
  }
}


void end()
{
  // Serial.print("\npath: ");
  // int c;
  // for (c=0; c<path_length; c++)
  // {
  //   Serial.print(path[c]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Serial.print("\nafter simplification: ");
  // for (c=0; c<path_length; c++)
  // {
  //   Serial.print(path[c]);
  //   Serial.print(" ");
  // }
  // Serial.println();
  pathSimplify();
  digitalWrite(LED, HIGH);

  while(!digitalRead(finalSwitch))
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    delay(100);
  }
  digitalWrite(LED, LOW);
  finalFlag = 1;
  return;
}

void end2()
{
  digitalWrite(LED, HIGH);
  while(1)
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    delay(100);
  }
}


void calibrate() {
  Serial.println("Start Calibrating ");
  for (int i = 0; i < 1000; i++) {
    for (int j = 0; j <=6; j++) {
      int val = analogRead(pins[j]);
      maxi[j] = (val > maxi[j]) ? val : maxi[j];
      mini[j] = (val < mini[j]) ? val : mini[j];
    }
    delay(10);
  }
  digitalWrite(LED, HIGH);
  while(digitalRead(finalSwitch))
  {
    stop(10);
  }
  digitalWrite(LED, LOW);
 
}




void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(algoSwitch, INPUT_PULLUP);
  pinMode(finalSwitch, INPUT_PULLUP);

  if(digitalRead(algoSwitch))
    algoFlag = 1;

  if(digitalRead(finalSwitch))
   {calibrate();
   }

}

void loop() {

  readArray();
  delay(10);
 
    followline();

  // Serial.println("followline");
  // }
  //moveforward(30);
  if (finalFlag)
  {
    if (node == path_length)
      end2();

    switch(path[node++]){
      case 'R':
        rightTurn(10);
        break;
      case 'L':
        leftTurn(10);
        break;
      case 'S':
        break;
    }
  }
  else{
    if (algoFlag)
      rightfirst();
    else
      leftfirst();

  }
}