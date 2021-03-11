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
#define STP_DIR RST
#define EMPTY 13
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

void setup() {
  // put your setup code here, to run once:
  pinMode(ANALOG_INPUT_POT, INPUT);
  pinMode(ANALOG_INPUT_SPEED, INPUT);
  pinMode(ADC2, INPUT);         // CAN ALSO BE OUTPUT
  pinMode(ADC3, INPUT);         // CAN ALSO BE OUTPUT
  pinMode(ADC4_SDA, INPUT);     // CAN ALSO BE OUTPUT
  pinMode(SCL, INPUT);          // CAN ALSO BE OUTPUT
  pinMode(OUTPUT_LED, OUTPUT);          
  pinMode(STP_EN, OUTPUT);        
  pinMode(STP_DIR, OUTPUT);
  pinMode(STP_STEP, OUTPUT);
  pinMode(EMPTY, OUTPUT);       // NOT USED
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

  Serial.begin(9600);

  // Configure interrupt for sensor
  attachInterrupt(digitalPinToInterrupt(SENSOR_IN), SENSOR_INTERRUPT, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("It's alive!");
}

void SENSOR_INTERRUPT()
{

}