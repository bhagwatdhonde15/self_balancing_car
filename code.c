SELF BAL 
:-
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// ---------- WiFi ----------
const char* ssid = "Bhagwat";
const char* password = "bhagwat@15";

ESP8266WebServer server(80);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ---------- Motor Pins ----------
#define IN1 D5
#define IN2 D6
#define IN3 D7
#define IN4 D8

// ---------- PID ----------
float kp = 35.0;
float ki = 1.0;
float kd = 3.0;

float targetAngle = 0;
float integral = 0;
float lastError = 0;
float angleOffset = 0;

unsigned long previousMicros = 0;

// ---------- Variables ----------
float angle = 0;
float control = 0;

// ---------- Motor Function ----------
void driveMotors(float output)
{
  int pwm = constrain(abs((int)output), 0, 255);

  if (output > 0)
  {
    analogWrite(IN1, pwm);
    analogWrite(IN2, 0);

    analogWrite(IN3, pwm);
    analogWrite(IN4, 0);
  }
  else
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, pwm);

    analogWrite(IN3, 0);
    analogWrite(IN4, pwm);
  }
}

// ---------- Web ----------
void handleRoot()
{
  String page = R"====(
<!DOCTYPE html>
<html>
<head>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<style>
body{
font-family:Arial;
text-align:center;
background:#111;
color:white;
}

button{
width:150px;
height:60px;
font-size:24px;
margin:10px;
border-radius:15px;
}
</style>
</head>

<body>

<h1>Self Balancing Car</h1>

<button onclick="location.href='/front'">FRONT</button>
<br>
<button onclick="location.href='/stop'">STOP</button>
<br>
<button onclick="location.href='/back'">BACK</button>

</body>
</html>
)====";

  server.send(200, "text/html", page);
}

void handleFront()
{
  targetAngle = 3.0;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleBack()
{
  targetAngle = -3.0;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStop()
{
  targetAngle = 0.0;
  server.sendHeader("Location", "/");
  server.send(303);
}

// ---------- Setup ----------
void setup()
{
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWriteFreq(5000);
  analogWriteRange(255);

  Wire.begin(D2, D1);

  if (!accel.begin())
  {
    Serial.println("ADXL345 NOT FOUND");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_2_G);

  // Calibration
  Serial.println("Calibrating...");

  for (int i = 0; i < 300; i++)
  {
    sensors_event_t event;
    accel.getEvent(&event);

    float a =
      atan2(event.acceleration.x,
            event.acceleration.z)
      * 180.0 / PI;

    angleOffset += a;

    delay(5);
  }

  angleOffset /= 300.0;

  Serial.print("Offset = ");
  Serial.println(angleOffset);

  // WiFi
  WiFi.begin(ssid, password);

  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/front", handleFront);
  server.on("/back", handleBack);
  server.on("/stop", handleStop);

  server.begin();

  previousMicros = micros();
}

// ---------- Loop ----------
void loop()
{
  server.handleClient();

  sensors_event_t event;
  accel.getEvent(&event);

  angle =
      atan2(event.acceleration.x,
            event.acceleration.z)
      * 180.0 / PI
      - angleOffset;

  unsigned long now = micros();

  float dt =
      (now - previousMicros)
      / 1000000.0;

  previousMicros = now;

  float error =
      targetAngle - angle;

  integral += error * dt;

  integral =
      constrain(integral,
                -100,
                100);

  float derivative =
      (error - lastError)
      / dt;

  control =
      kp * error +
      ki * integral +
      kd * derivative;

  lastError = error;

  driveMotors(control);

  Serial.print("Angle: ");
  Serial.print(angle);

  Serial.print("  Target: ");
  Serial.print(targetAngle);

  Serial.print("  PID: ");
  Serial.println(control);

  delay(5);
}
