#include <Wire.h>
#include <LSM303.h>
#include <TinyGPS.h> //Connect with pin 18 and 19 rx = 19, tx = 18
#include <Servo.h>
#include <LiquidCrystal_I2C.h> //Load Liquid Crystal Library

/*
   Notes: Use LEDs, more efficient than LCD or Serial prints (use 220 ohm resistors)
   maybe use software serial if the gps continues to not work
*/

/*   90 is stop
   > 90 is forward
   < 90 is backwards
*/
Servo escServo;

/* around 100 is straight
   < 100 is turn right
   > 100 is turn left
*/
Servo wheelServo;

// GPS/Compass
TinyGPS gps; // create gps object
LSM303 compass; // create compass object
float lat1, long1; // for gps data
int headingCompass;
int headingTarget;
int headingError;
const int HEADING_TOLERANCE = 20;
int pCounter = 0;

// ------------------------------------------------------------------------------------------- Target lats and longs
String locStrings[] = {"Loc_1", "Loc_2", "Loc_3"};
float targetLat[] = {38.893390, 38.893180, 38.893150};
float targetLong[] = { -104.802350, -104.802490, -104.802100};

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars/row and 2 row/line display

// Compass Calibration
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};

// Sensor
int trigPin = 41;    // TRIG pin
int echoPin = 40;    // ECHO pin
float duration_us, distance_cm;
float stopDist = 70.0; // 2 feet is ~ 62 cm

// ---------------------------------------------------------------------------------------------------------------------------
void setup() {
  // Setup serials for debugging
  Serial.begin(57600);

  // Setup servos
  wheelServo.attach(8);
  wheelServo.write(100); // start straight
  escServo.attach(9); // attach to pwm pin 9

  // Setup Compass
  Wire.begin();
  compass.init();
  compass.enableDefault();
  /*
    Calibration values; the default values of +/-32767 for each axis
    lead to an assumed magnetometer bias of 0. Use the Calibrate example
    program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>) {
    32767, 32767, 32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    -32768, -32768, -32768
  };

  // Setup GPS
  Serial1.begin(9600); // connect gps sensor
  delay(100);

  // Setup LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);

  // Setup sensor
  pinMode(trigPin, OUTPUT); // config trigger pin to output mode
  pinMode(echoPin, INPUT);  // config echo pin to input mode

  delay(10000);
  lcd.print("Loc_0");
  drive('f');
  pCounter = 0;
}

// ---------------------------------------------------------------------------------------------------------------------------
void loop() {

  //distSensor();
  //if (distance_cm < stopDist)
    //avoidObstacle();
  compassRun();
  gpsRun();
  determineDirection();

  //lat1 >= 38.89337 && lat1 <= 38.893392 && long1 <= -104.80233 && long1 >= -104.80237 // note: if off, check upper bound of lat1
  // stop if within error circle
  if (lat1 >= targetLat[pCounter] - .000020 && lat1 <= targetLat[pCounter] + .000020 && long1 <= targetLong[pCounter] + .000020 && long1 >= targetLong[pCounter] - .000020) {
    drive('s');
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(locStrings[pCounter]);
    delay(10000);
    if (pCounter == 2) {
      // Spiral search
      int spiral = 150;
      drive('f');
      while (1) {
         // generate 10-microsecond pulse to TRIG pin
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
    
        // measure duration of pulse from ECHO pin
        duration_us = pulseIn(echoPin, HIGH);
    
        // calculate the distance
        distance_cm = 0.017 * duration_us;
        delay(100);
        if (distance_cm < stopDist) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Done");
          exit(0);
        }
        wheelServo.write(spiral);
        spiral -= 5;
        delay(5000);
        if (spiral <= 110)
          spiral = 110;
      }
    }
    pCounter++;
    drive('f');
  }

  distSensor();
  //delay(50);
}

// ---------------------------------------------------------------------------------------------------------------------------
// Function for determing which way the truck needs to go based on target coordinates and current coordinates as well as compass
// From: https://www.instructables.com/Arduino-Powered-Autonomous-V ehicle/
void determineDirection() {
  // for now use just lat1 and long1, it should be getting updated about every second
  headingTarget = gps.course_to(lat1, long1, targetLat[pCounter], targetLong[pCounter]);

  // calculate where we need to turn to head to destination
  headingError = headingTarget - headingCompass;

  // adjust for compass wrap
  if (headingError < -180)
    headingError += 360;
  if (headingError > 180)
    headingError -= 360;

  // calculate which way to turn to intercept the targetHeading
  if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
    turn('s');
  else if (headingError < 0)
    turn('l');
  else if (headingError > 0)
    turn('r');
  else
    turn('s');
}

// ---------------------------------------------------------------------------------------------------------------------------
// GPS function
void gpsRun() {
  while (Serial1.available()) { // check for gps data
    if (gps.encode(Serial1.read())) // encode gps data
    {
      gps.f_get_position(&lat1, &long1); // get latitude and longitude
      return;
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
// Compass function
void compassRun() {
  compass.read();

  /*
    When given no arguments, the heading() function returns the angular
    difference in the horizontal plane between a default vector and
    north, in degrees.

    The default vector is chosen by the library to point along the
    surface of the PCB, in the direction of the top of the text on the
    silkscreen. This is the +X axis on the Pololu LSM303D carrier and
    the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
    carriers.

    To use a different vector as a reference, use the version of heading()
    that takes a vector argument; for example, use

    compass.heading((LSM303::vector<int>){0, 0, 1});

    to use the +Z axis as a reference.
  */
  headingCompass = compass.heading();

  delay(100);
}

// ---------------------------------------------------------------------------------------------------------------------------
// Check the distance in front
void distSensor() {
  /*
  int  arraySize = 30;
  float valueArray[arraySize];
  float total = 0;
  for (int i = 0; i < arraySize; i++) {
    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(echoPin, HIGH);

    // calculate the distance
    distance_cm = 0.017 * duration_us;

    valueArray[i] = distance_cm;
  }

  for(int i = 0; i < arraySize; i++){
    total += valueArray[i];
  }

  distance_cm = total/arraySize;
  */

  int counter = 0;
  int amountLessThan = 0;
  while(counter < 3){
    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(echoPin, HIGH);

    // calculate the distance
    distance_cm = 0.017 * duration_us;

    if(distance_cm < stopDist)
      amountLessThan++;

    counter++;
    delay(80); //delay(100);
  }

  if(amountLessThan >= 2)
    avoidObstacle();
  //else
    //distance_cm = stopDist + 5;
  //delay(100);
}

void avoidObstacle() {
  drive('s');
  turn('s');
  delay(500);
  drive('b');
  delay(2000);
  turn('r');
  delay(2000);
  drive('s');
  turn('s');
  delay(500);
  drive('f');
  turn('l');
  delay(4000);
  turn('s');
  delay(500);
}

// ---------------------------------------------------------------------------------------------------------------------------
// Sets the motor to go based on argument, f for forward, b for backwards, s for stop
void drive(char c) {
  switch (c) {
    case 'f':
      escServo.write(110);
      break;
    case 'b':
      escServo.write(70);
      break;
    case 's':
      escServo.write(90);
      break;
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
// Sets the wheels to turn based on argument, l for left, r for right, s for straight
void turn(char c) {
  switch (c) {
    case 'l':
      wheelServo.write(130); // 125
      break;
    case 'r':
      wheelServo.write(75); // 80
      break;
    case 's':
      wheelServo.write(100);
      break;
  }
}
