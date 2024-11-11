#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
#include <L298N.h>

int lastError = 0;
boolean onoff = 0;
int val, cnt = 0, v[3];

const uint16_t threshold = 1500; // adjustable - can take values between 0 and 1000

//the speed can be between 0 and 255 - 0 is LOW and 255 is HIGH. At a high value, 
//you may risk burning the motors, because the voltage supplied by the PWM control
//is higher than 6V.
const int maxspeeda = 255;
const int maxspeedb = 255;
const int basespeeda = 100;
const int basespeedb = 100;
/*If your robot can't take tight curves, you can set up the robot
  to revolve around the base (and not around one of the wheels) by
  setting the minspeed to a negative value (-100), so the motors will go 
  forward and backward. Doing this the motors can wear out faster. 
  If you don't want to do this, set the minspeed to 0, so the motors 
  will only go forward.
*/
//const int minspeeda = 0;
//const int minspeedb = 0;
const int minspeeda = -100;
const int minspeedb = -100;

float Kp = 0;
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
int P;
int I;
int D;
float Pvalue;
float Ivalue;
float Dvalue;

int motor1pin1 = 5;
int motor1pin2 = 3;
int ENA = 9;

int motor2pin1 = 2;
int motor2pin2 = 4;
int ENB = 10;

L298N motor1(ENA, motor1pin1, motor1pin2);
L298N motor2(ENB, motor2pin1, motor2pin2);


void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14, 15, 16, 18, 19}, SensorCount);
  
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  calibration();
  forward_brake(0, 0);
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial.available()) {
    while(Serial.available() == 0);
    valuesread();
    processing();
  }
  if(onoff == 1) {
    robot_control();
  }
  if(onoff == 0) {
    forward_brake(0, 0);
  }
}

//This void delimits each instruction.
//The Arduino knows that for each instruction it will receive 2 bytes.
void valuesread() {
  val = Serial.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
}

//In this void the the 2 read values are assigned.
void processing() {
  int a = v[1];
  if (a == 1) {
    Kp = v[2];
  }
  if (a == 2) {
    multiP = v[2];
  }
  if (a == 3) {
    Ki = v[2];
  }
  if (a == 4) {
    multiI = v[2];
  }
  if (a == 5) {
    Kd = v[2];
  }
  if (a == 6) {
    multiD = v[2];
  }
  if (a == 7) {
    onoff = v[2];
  }
}

//Make sure that this values are assigned correctly
void forward_brake(int posa, int posb) {
  //digitalWrite(aphase, LOW);
  //digitalWrite(bphase, HIGH);
  motor1.setSpeed(posa);
  motor2.setSpeed(posb);

  motor1.forward();
  motor2.forward();
}
void left_brake(int posa, int posb) {
  //digitalWrite(aphase, LOW);
  //digitalWrite(bphase, LOW);
  motor1.setSpeed(posa);
  motor2.setSpeed(posb);

  motor1.backward();
  motor2.forward();
}
void right_brake(int posa, int posb) {
  //digitalWrite(aphase, HIGH);
  //digitalWrite(bphase, HIGH);
  motor1.setSpeed(posa);
  motor2.setSpeed(posb);

  motor1.forward();
  motor2.backward();

}

void robot_control() {
  uint16_t position = qtr.readLineWhite(sensorValues);
  int error = 2000 - position;

  int cnt = 0;
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    if(sensorValues[i] >= threshold) {
      cnt++;
      sum = sum + i;
    }
  }

  
  //I made a case where the robot can cross the line (when the curve is too tight) and the position can be 3500
  //even though it is not on the center of the line. If you don't want your motors to rotate in 2 directions
  //comment the right_brake(100,100) / left_brake(100, 100) and uncomment the forward_brake(0,100) / forward_brake(0,100)

   
  if (cnt >= 2) {
    int motorspeeda = 0;
    int motorspeedb = 0;
    int val = sum/cnt;
    if(val < 2.0) {
      //turn right
      right_brake(100, 100);
      //forward_brake(0,100);
    }
    if(val > 2.0) {
      //turn left
      left_brake(100, 100);
      //forward_brake(100,0);
    }
    if(val == 2.0) {
      cnt = cnt/2;
      uint16_t mini = 1000;
      uint8_t minpos = 0;
      for (int i = 4 - cnt; i <= 3 + cnt; i++) {
         if (mini > sensorValues[i]) {
            mini = sensorValues[i];
            minpos = i;
         }
      }
      if(minpos < 2.0) {
        //turn right
        right_brake(100, 100);
        //forward_brake(0,100);
      }
      if(minpos > 2.0) {
        //turn left
        left_brake(100, 100);
        //forward_brake(100,0);
      }
    }
  }
  else {
    PID(error);
  }
}

void PID(int error) {
  int P = error;
  int I = I + error;
  int D = error - lastError;
  lastError = error;
  Pvalue = (Kp/pow(10,multiP))*P;
  Ivalue = (Ki/pow(10,multiI))*I;
  Dvalue = (Kd/pow(10,multiD))*D;

  float motorspeed = Pvalue + Ivalue + Dvalue;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < minspeeda) {
    motorspeeda = minspeeda;
  }
  if (motorspeedb < minspeedb) {
    motorspeedb = minspeedb;
  }
  //Serial.print(motorspeeda); Serial.print(" "); Serial.println(motorspeedb);
  speedcontrol(motorspeeda, motorspeedb);
}

void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    forward_brake(mota, motb);
  }
  if (mota < 0 && motb >= 0) {
    //dreapta
    mota = 0 - mota;
    right_brake(mota, motb);
     
  }
  if (mota >= 0 && motb < 0) {
    //stanga
    motb = 0 - motb;
    left_brake(mota, motb);
  }
}