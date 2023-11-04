// Replace 12345 with the correct team number and then uncomment the line below.
#define TEAM_NUMBER 15

#ifndef TEAM_NUMBER
#error "Define your team number with `#define TEAM_NUMBER 12345` at the top of the file."
#elif TEAM_NUMBER < 1 || 20 < TEAM_NUMBER
#error "Team number must be within 1 and 20"
#endif

#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 6;
int sensorValues[SensorCount];

// PID control loop values
const float Kp = 0.5;
const float Ki = 0.0;
const float Kd = 0.1;
float accum = 0.0;
float prevError = 0.0;
const float speed = 0.5;

bool auton_mode = false;

int servo2Rot = 90;

float blackLinePos(int sensorValues[SensorCount]) {
  int n = 0;
  int sum = 0;

  // Calculate minimum intensity in sensorValues
  int min = sensorValues[0];
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < min) min = sensorValues[i];
  }

  for (int i = 0; i < SensorCount; i++) {
    sum += i * (sensorValues[i] - min);
    n += sensorValues[i] - min;
  }
  // Return average index of values in sensorValues
  return sum / (1.0 * n);
}

void setMotors(float speed, float rot) {
  RR_setMotor1(speed - rot);
  RR_setMotor2(speed + rot);
}

void setup() {
  Serial.begin(115200);

  RR_setServo2(servo2Rot);

  pinMode(LED_BUILTIN, OUTPUT);
}

int temp = 0;

void loop() {
  RR_setMotor4(1.0);
  RR_setMotor2(1.0);
  return;
  // Get start and back buttons to switch between autonomous and manual modes
  bool btnStart = RR_buttonStart();
  bool btnBack = RR_buttonBack();

  if (btnStart) {
    auton_mode = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (btnBack) {
    auton_mode = false;
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (!auton_mode) {
    // Read the four joystick axes
    // These will be in the range [-1.0, 1.0]
    float rightX = RR_axisRX();
    float rightY = RR_axisRY();
    float leftX  = RR_axisLX();
    float leftY  = RR_axisLY();

    // Get the button states
    bool btnA = RR_buttonA();
    bool btnB = RR_buttonB();
    bool btnX = RR_buttonX();
    bool btnY = RR_buttonY();
    bool btnRB = RR_buttonRB();
    bool btnLB = RR_buttonLB();
    bool btnRT = RR_buttonRT();
    bool btnLT = RR_buttonLT();

    // Motor sensitivity
    float k = btnRT ? 0.25 : 1.0;

    // Arcade-drive scheme
    // Left Y-axis = throttle
    // Right X-axis = steering
    setMotors(k * leftY, k * rightX);

    // Control servo 2 using the shoulder buttons
    if (btnLB && servo2Rot < 135) servo2Rot += 2;
    else if (btnRB && servo2Rot > 45) servo2Rot -= 2;
    RR_setServo2(servo2Rot);

    // we also have RR_setServo3 and RR_setServo4 available
  }
  else if (auton_mode) {
    // read the ultrasonic sensors
    // Serial.print("Ultrasonic=");
    // Serial.print(RR_getUltrasonic());
    // Serial.print(" ;; ");
    // int sensors[6];

    // Serial.print("Line sensors=");
    // RR_getLineSensors(sensorValues);
    // for (int i = 0; i < 6; ++i) {
    //   Serial.print(sensorValues[i]);
    //   Serial.print(" ");
    // }
    // Serial.print("\tLine pos=");
    // Serial.print(blackLinePos(sensorValues));
    // Serial.println();

    float error = blackLinePos(sensorValues) - 2.5;
    accum += error * 0.02;
    float deriv = (error - prevError) / 0.02;
    float adjustment = Kp*error + Ki*accum + Kd*deriv;
    prevError = error;

    setMotors(speed, adjustment*speed);
  }

  // This is important - it sleeps for 0.02 seconds (= 50 times / second)
  // Running the code too fast will overwhelm the microcontroller and peripherals
  delay(20);
}

// vim: tabstop=2 shiftwidth=2 expandtab
