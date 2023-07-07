int sensor1 = 18;      // Left most sensor
int sensor2 = 2;
int sensor3 = 3;
int sensor4 = 19;      // Right most sensor

// Initial Values of Sensors
int sensor[4] = {0, 0, 0, 0};
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
// TCS230 pins connected to Arduino
const int s0 = 26;
const int s1 = 28;
const int s2 = 32;
const int s3 = 30;
const int out = 34;

// Variables
int red = 0;
int green = 0;
int blue = 0;


//Initial Speed of Motor
int initial_motor_speed = 140;

// PID Constants
float Kp = 23;
float Ki = 2;
float Kd = 15;

//motor specification
int MAXSPEED = 255; // MAX 255
int MINSPEED = 0; // MAX 255
int AVRAGE = 128;
int turn_delay = 2000;
int left_motor_speed,right_motor_speed;
//motor specification
const int motorright2 = 9;
const int motorright1 = 8;
const int rightspeed = 7;
const int motorleft2 = 10;
const int motorleft1 = 11;
const int leftspeed = 12;
// counter
volatile int  i =0,ir=0, j = 1;
int leftnew = 0, rightnew = 0;
int leftpast = 0, rightpast = 0;
//sensor
long sensors[] = {0, 0, 0, 0, 0};

// put your setup code here, to run once:
void setup() {
  Serial.begin(9600);
  //turn on pullup resistor
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  pinMode(motorright1, OUTPUT);
  pinMode(motorright2, OUTPUT);
  pinMode(rightspeed, OUTPUT);
  pinMode(motorleft1, OUTPUT);
  pinMode(motorleft2, OUTPUT);
  pinMode(leftspeed, OUTPUT);
   pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  leftpast = digitalRead(sensor1);
  rightpast = digitalRead(sensor4);
  attachInterrupt(0, counter, CHANGE);
 
}
// put your main code here, to run repeatedly:
void loop() {
 if(i>=0 && i<9){
  read_sensor_values();
  if (error == 0 ) {
  analogWrite (rightspeed, initial_motor_speed-20);
  analogWrite (leftspeed, initial_motor_speed);
  digitalWrite(motorright2, HIGH);
  digitalWrite(motorright1, LOW);
  digitalWrite(motorleft2, HIGH);
  digitalWrite(motorleft1, LOW);
    //delay(200);
    }  
  else {
    read_sensor_values();
    calculate_pid();
    motor_control();
  //analogWrite (rightspeed, initial_motor_speed-25);
  //analogWrite (leftspeed, initial_motor_speed);
  digitalWrite(motorright2, HIGH);
  digitalWrite(motorright1, LOW);
  digitalWrite(motorleft2, HIGH);
  digitalWrite(motorleft1, LOW);
//    delay(100);
    }  }
 if (j % 2 == 0 ){ 
  if(i==9){
        delay(250);
        analogWrite (leftspeed,MAXSPEED);
        analogWrite (rightspeed,MINSPEED );
        turnright();
        delay(2000);}
else if (i==10){
        analogWrite (leftspeed,MAXSPEED-20);
        analogWrite (rightspeed,MINSPEED );
        turnright();
        delay(2000);}
 }

 if (j % 2 == 1 ){ 
  if(i == 9){
        delay(250);
        analogWrite (rightspeed,MAXSPEED);
        analogWrite (leftspeed,MINSPEED );
        turnleft();
        delay(2000);
        }
 else if(i==10){
        analogWrite (rightspeed,MAXSPEED);
        analogWrite (leftspeed,MINSPEED );
        turnleft();
        delay(2500);
       }}
       
       color();
  //Serial.print("i=");
  //Serial.println(i);
  colordetect();
}

void counter() {
  leftnew = digitalRead(sensor2);
  if (leftnew == 0 && leftnew != leftpast ) {
    i = i + 1;

    delayMicroseconds(250);
    //pulse=micros();
     if(i==12){
      i=2;
     j=j+1;}
  }
  else
  { //delayMicroseconds(250);
    leftpast = leftnew;
  }
}
void counterj() {
  rightnew = digitalRead(sensor3);
  if (rightnew == 0 && rightnew != rightpast ) {
    ir = ir + 1;
   
    delayMicroseconds(250);
   
    //pulse=micros();
  }
  else
  { //delayMicroseconds(250);
    rightpast = rightnew;
  }
}

int read_sensor_values()
{
  sensor[0] = !digitalRead(sensor1);
  sensor[1] = !digitalRead(sensor2);
  sensor[2] = !digitalRead(sensor3);
  sensor[3] = !digitalRead(sensor4);

  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0))
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1))
    error = -2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1))
    error = -3;
 else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1)) // Turn robot left side
    error = -4;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) // Turn robot left side
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Turn robot right side
    error = 101;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // Make U turn
    error = 0;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Turn left side or stop
    error = 0;
    return error;}
//  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0))
//   error = 0;



void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 220);

  /*Serial.print(PID_value);
    Serial.print("\t");
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.println(right_motor_speed);*/

  analogWrite(leftspeed, left_motor_speed); //Left Motor Speed
  analogWrite(rightspeed, right_motor_speed ); //Right Motor Speed
}

void turnright() {
  //analogWrite (leftspeed, left_motor_speed);
  digitalWrite (motorleft2, HIGH);
  digitalWrite(motorleft1, LOW);
  //delay(turn_delay);
  //analogWrite (rightspeed, right_motor_speed);
  digitalWrite (motorright2, LOW);
  digitalWrite(motorright1, HIGH);
  // delay(turn_delay);
}
void turnleft() {
 
  digitalWrite (motorright2, HIGH);
  digitalWrite(motorright1, LOW);
  //delay(turn_delay);
  digitalWrite (motorleft2, LOW);
  digitalWrite(motorleft1, HIGH);
 // delay(2000);
  }

void color()
{
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  //count OUT, pRed, RED
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  //count OUT, pGreen, GREEN
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}
void colordetect(){
  if (red < blue && red < green && red < 25)
  {
    if (green - blue >= 10 && green - blue <= 25 && green - ( 2 * red ) >= 8 )
    {
      //Serial.println(" - (Red Color)");
    }

   else if (green - red <= 10 && green - red >= -3 && blue >= green)
    {
      //Serial.println(" - (Yellow Color)");
    }
    
    else if (blue - red >= 3 && blue - red <= 10 &&  green - ( 2 * red ) <= 5)
    {Serial.print("i=");
     Serial.print(i);
     Serial.print("j=");
     Serial.print(j);
      Serial.println(" - (Pink Color)");
    }
    
    else if (green - blue >= -5 && green - blue <= 5 && green - ( 2 * red ) <= 5 )
    {
//Serial.print("i=");
//     Serial.print(i);
//      Serial.println(" - (pink Color)");
    }

  }

  else if (green < red && green < blue && green < 25)
  {Serial.print("i=");
     Serial.print(i);
     Serial.print("j=");
     Serial.print(j);
    Serial.println(" - (Green Color)");
  }

  else if ((red > green &&  blue < green) && blue < 25 && red > 40)
  {  //Serial.print("i=");
    // Serial.print(i);
    //Serial.println(" - (Blue Color)");
  }

  else if (red - (2 * blue) >= -2 && red - (2 * blue) <= 5 && green - red < 10)
  {
    //Serial.println(" - (Purple Color)");
  }
  else if (blue < red && blue < green && (blue && red && green) < 25)
  {

    if (red - green <= 5 && red - green >= 0 && ((green - blue) || (red - blue)) < 5 && blue < 50)
    {
      //Serial.println(" - (White Color)");
    }
  }

  //Serial.println();

  delay(100);
}
