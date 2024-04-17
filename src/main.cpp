#include <Arduino.h>
#include <SPI.h>

#include <mcp_can.h>
#include <analogsensor.h>
#include <currentsensor.h>
#include <servo.h>
/*pwm pins:
Leonardo, Micro, Yún

3, 5, 6, 9, 10, 11, 13

490 Hz (pins 3 and 11: 980 Hz)
*/

#define LEFT_SHUNT_PIN A0
#define RIGHT_SHUNT_PIN A1
const int current_sensors_cutoff_freq = 10;
analogSensor adc_shunt_left = analogSensor(analogRead, LEFT_SHUNT_PIN, current_sensors_cutoff_freq);
analogSensor adc_shunt_right = analogSensor(analogRead, RIGHT_SHUNT_PIN, current_sensors_cutoff_freq);

const float current_sensors_scale = 0.5;
current_sensor shunt_left = current_sensor(&adc_shunt_left, current_sensors_scale);
current_sensor shunt_right = current_sensor(&adc_shunt_right, current_sensors_scale);

const float SERVO_CURRENT_LIM = 3.0;
const float servos_angle_range[] = {0, 180};
servo leftServo = servo(5, &shunt_left, servos_angle_range[0], servos_angle_range[1], false);
servo rightServo = servo(6, &shunt_right, servos_angle_range[0], servos_angle_range[1], true);

servo *servos[] = {&leftServo, &rightServo};
MCP_CAN can = MCP_CAN(10);

const unsigned long superloop_debug = 500;
unsigned long last_time = 0;
void setup()
{
  // Init Serial
  Serial.begin(9600);

  // Init GPIOs
  pinMode(adc_shunt_left.getPin(), INPUT);
  pinMode(adc_shunt_right.getPin(), INPUT);

  // Init MCP
  Serial.println(can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ));
  can.setMode(MCP_NORMAL);

  // Init Servos
  for (int i = 0; i < sizeof(servos) / sizeof(servos[0]); i++)
  {
    servos[i]->init();
  }
  last_time = millis();
}

void loop()
{
  for (int i = 0; i < sizeof(servos) / sizeof(servos[0]); i++)
  {
    servos[i]->run();
  }
  if (millis() - last_time < superloop_debug)
  {
    Serial.print("Left Servo urrent: ");
    Serial.println(shunt_left.getCurrent());
    Serial.print("Right Servo Current: ");
    Serial.println(shunt_right.getCurrent());
    last_time = millis();
  }
  if (Serial.available() > 0)
  {                                             // Check if data is available to read
    String data = Serial.readStringUntil('\n'); // Read the data until newline character is received
    float number = data.toFloat();              // Convert the received string to float

    // Print the received float number
    Serial.print("Received number: ");
    Serial.println(number);
    for (int i = 0; i < sizeof(servos) / sizeof(servos[0]); i++)
    {
      servos[i]->setAngle(number);
    }
    // You can now use the 'number' variable in your Arduino program
  }
}