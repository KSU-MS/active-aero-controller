#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <mcp_can.h>
#include <analogsensor.h>
#include <currentsensor.h>
#include <ksu_servo.h>
#include <carstate.h>
#include <ksu_ev_can.h>
/*pwm pins:
Leonardo, Micro, Yún

3, 5, 6, 9, 10, 11, 13

490 Hz (pins 3 and 11: 980 Hz)
*/
static can_obj_ksu_ev_can_h_t ksu_can;
#define LEFT_SHUNT_PIN A1
#define RIGHT_SHUNT_PIN A0
const int current_sensors_cutoff_freq = 10;
analogSensor adc_shunt_left = analogSensor(analogRead, LEFT_SHUNT_PIN, current_sensors_cutoff_freq);
analogSensor adc_shunt_right = analogSensor(analogRead, RIGHT_SHUNT_PIN, current_sensors_cutoff_freq);

const float current_sensors_scale = 10; // 0.1v per amp
// https://www.digikey.com/en/products/detail/allegro-microsystems/ACS712ELCTR-20A-T/1284594

current_sensor shunt_left = current_sensor(&adc_shunt_left, current_sensors_scale, -2.45);
current_sensor shunt_right = current_sensor(&adc_shunt_right, current_sensors_scale, -2.45);

const float SERVO_CURRENT_LIM = 5.0;
const float servos_angle_range[] = {0, 180};
Servo leftservo_;
Servo rightservo_;
ksu_servo leftServo = ksu_servo(&leftservo_, 5, &shunt_left, servos_angle_range[0], servos_angle_range[1], false);
ksu_servo rightServo = ksu_servo(&rightservo_, 6, &shunt_right, servos_angle_range[0], servos_angle_range[1], true);

drs_system drs = drs_system(&leftServo, &rightServo);

MCP_CAN can = MCP_CAN(10);

const unsigned long superloop_debug = 2000;
unsigned long last_time = 0;

int poll_can();
void setup()
{
  // Init GPIOs
  pinMode(adc_shunt_left.getPin(), INPUT);
  pinMode(adc_shunt_right.getPin(), INPUT);
  drs.init();

  // Init Serial
  Serial.begin(9600);
  // Serial.setTimeout(0);

  // Init MCP
  Serial.println(can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ));
  can.setMode(MCP_NORMAL);

  // Calibrate DRS
  drs.calibrate();
  // drs.setAngle(DRS_MIDDLE_ANGLE_HARDCODED);
  last_time = millis();
}
/* Joe Aero Pseudo Code
Function Car_startup()
  DRS_pos = 100
  DRS_pos = 0

Function drs(steering_angle, gas_pot, brake_pot)
  If(steering_angle <= 15 deg AND brake_pot <= 5% AND gas_pot => 5%){

  DRS_pos = 100

  }Else{

  DRS_pos = 0

}
*/
float angle = 0;
bool closed = false;
void loop()
{
  // poll_can();
  drs.updateCurrents();
  // if (driver_inputs.bse1_travel > 20)
  // {
  //   drs.setClosed();
  // }
  // else if (driver_inputs.bse1_travel < 20 && driver_inputs.apps1_travel > 50)
  // {
  //   drs.setOpen();
  // }

  if (millis() - last_time > superloop_debug)
  {
    // Serial.print("Left Servo Current: ");
    // Serial.println(shunt_left.getCurrent());
    if (closed)
    {
      drs.setOpen();
      closed = false;
    }
    else if (!closed)
    {
      drs.setClosed();
      closed = true;
    }
    last_time = millis();
  }

  if (Serial.available() > 0)
  {                                             // Check if data is available to read
    String data = Serial.readStringUntil('\n'); // Read the data until newline character is received
    float number = data.toFloat();              // Convert the received string to float

    // Print the received float number
    Serial.print("Received number: ");
    Serial.println(number);
    drs.setAngle(number);
  }
  // Serial.println(shunt_right.getCurrent());
}

int poll_can()
{
  static long unsigned int rxId;
  static unsigned char len = 0;
  static unsigned char rxBuf[8];
  if (can.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK)
  {
    uint64_t data;
    memcpy(&data, rxBuf, sizeof(rxBuf));
    unpack_message(&ksu_can, rxId, data, len, millis());
    switch (rxId)
    {
    case (CAN_ID_M192_COMMAND_MESSAGE):
    {
      decode_can_0x0c0_Torque_Command(&ksu_can, &driver_inputs.torque_command);
      break;
    }
    case (CAN_ID_VCU_PEDAL_READINGS):
    {
      decode_can_0x0c4_BSE1(&ksu_can, &driver_inputs.bse_voltage);
      break;
    }
    case (CAN_ID_VCU_PEDALS_TRAVEL):
    {
      decode_can_0x0cc_vcu_apps1_travel(&ksu_can, &driver_inputs.apps1_travel);
      decode_can_0x0cc_vcu_apps2_travel(&ksu_can, &driver_inputs.apps2_travel);
      decode_can_0x0cc_vcu_bse1_travel(&ksu_can, &driver_inputs.bse1_travel);

      break;
    }
    default:
    {
      break;
    }
    }
  }
  else
  {
    return 0;
  }
  return 0;
}