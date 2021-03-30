#include <Arduino.h>

// PinMap
#define ANALOG_INPUT_POT A0
#define ANALOG_INPUT_SPEED A1
#define ADC2 A2
#define ADC3 A3
#define ADC4_SDA A4
#define SCL A5
#define OUTPUT_LED A6
#define STP_EN A7
#define STP_DIR 13
//#define EMPTY 13
#define STP_STEP 12
#define DCMOTOR3_INH 11
#define DCMOTOR2_INH 10
#define EN_PWM 9
#define DCMOTOR1_INH 8
#define DIR_RS485 7
#define DCMOTOR3_IN 6
#define DCMOTOR2_IN 5
#define DIRECTION_INPUT 4
#define DCMOTOR1_IN 3
#define SENSOR_IN 2
#define RX_RS485 0
#define TX_RS485 1

// LED RELATED VARIABLES
int brightness = 0;    // how bright the LED is
int fadeAmount = 20;    // how many points to fade the LED by

// STEPPER MOTOR RELATED VARIABLES
long int _speed=1000;
long int position_offset=1600;


void rotate_motor(long int steps_second)
{
  cli();
  Serial.println("Rotating motor!");
  Serial.println("-------------------------");
  
  for (int i=0; i<steps_second; i++)
  {
    digitalWrite(STP_STEP,HIGH);
    delayMicroseconds(_speed);
    digitalWrite(STP_STEP,LOW);
    delayMicroseconds(_speed);
  }
  Serial.println("Finished rotating!");
  sei();
}

void change_direction(bool direction)
{
  delayMicroseconds(1000);
  digitalWrite(STP_EN,HIGH);
  digitalWrite(STP_DIR,direction);
  digitalWrite(STP_EN,LOW);
  delayMicroseconds(1000);
}

void SENSOR_INTERRUPT()
{
  Serial.println("Sensor triggered!");
  
  digitalWrite(EN_PWM, HIGH);
  analogWrite(EN_PWM, 200);
  digitalWrite(EN_PWM, LOW);
}

void setup() {
  // VARIABLES CODE
  pinMode(ANALOG_INPUT_POT, INPUT);
  pinMode(ANALOG_INPUT_SPEED, INPUT);
  pinMode(ADC2, INPUT);         // CAN ALSO BE OUTPUT
  pinMode(ADC3, INPUT);         // CAN ALSO BE OUTPUT
  pinMode(ADC4_SDA, INPUT);     // CAN ALSO BE OUTPUT
  pinMode(SCL, INPUT);          // CAN ALSO BE OUTPUT
  pinMode(OUTPUT_LED, OUTPUT);          
  pinMode(STP_EN, OUTPUT);        
  //pinMode(STP_DIR, OUTPUT);
  pinMode(STP_STEP, OUTPUT);
  pinMode(STP_DIR, OUTPUT);       // NOT USED
  pinMode(DCMOTOR3_INH, OUTPUT);
  pinMode(DCMOTOR3_IN, OUTPUT);
  pinMode(DCMOTOR2_INH, OUTPUT);
  pinMode(DCMOTOR2_IN, OUTPUT);
  pinMode(DCMOTOR1_INH, OUTPUT);
  pinMode(DCMOTOR1_IN, OUTPUT);
  pinMode(EN_PWM, OUTPUT);
  pinMode(DIR_RS485, OUTPUT);
  pinMode(DIRECTION_INPUT, INPUT);
  pinMode(SENSOR_IN, INPUT);
  //pinMode(RX_RS485, INPUT);       // NOT USED
  //pinMode(TX_RS485, OUTPUT);       // NOT USED

  // SET PWM TO OFF
  digitalWrite(EN_PWM, LOW);

    // SERIAL PORT CODE
  Serial.begin(115200);

  // INTERRUPT CODE
  attachInterrupt(digitalPinToInterrupt(SENSOR_IN), SENSOR_INTERRUPT, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read the value from the sensor:
  brightness = analogRead(ANALOG_INPUT_POT);
  digitalWrite(STP_DIR, HIGH);
  delay(brightness);
  digitalWrite(STP_DIR, LOW);
  delay(brightness);
  Serial.println(brightness);
  Serial.println("It's alive!");
  delay(500);
  
  digitalWrite(EN_PWM, HIGH);

  analogWrite(EN_PWM, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 120) {
    fadeAmount = -fadeAmount;
  }

  delay(500);

  
  Serial.print("Brightness: ");
  Serial.println(brightness);
  //digitalWrite(EN_PWM, LOW);

  change_direction(HIGH);

  rotate_motor(position_offset);

  change_direction(LOW);
  
  rotate_motor(position_offset);
}

