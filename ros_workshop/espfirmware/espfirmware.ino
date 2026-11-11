
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// =====================================================
// 1. WIFI CONFIGURATION
// =====================================================

const char* WIFI_SSID = "Aaa";
const char* WIFI_PASSWORD = "1234567890";

WiFiUDP udp;

const uint16_t UDP_PORT = 8888;

IPAddress laptopIP;
uint16_t laptopPort = 0;

bool laptopKnown = false;
bool udpStarted = false;

// =====================================================
// 2. MOTOR PIN CONFIGURATION
// =====================================================

// Left motor
#define IN1 18
#define IN2 19
#define ENA 23

// Right motor
#define IN3 21
#define IN4 22
#define ENB 25

// Set true if the corresponding motor rotates
// in the opposite direction.
#define LEFT_MOTOR_INVERTED false
#define RIGHT_MOTOR_INVERTED false

// =====================================================
// 3. PWM CONFIGURATION
// =====================================================

#define PWM_FREQ 15000
#define PWM_RESOLUTION 8

#define PWM_MAX 255
#define MIN_MOTOR_PWM 65

bool pwmLeftReady = false;
bool pwmRightReady = false;

// =====================================================
// 4. ULTRASONIC SENSOR CONFIGURATION
// =====================================================

#define TRIG_PIN 26
#define ECHO_PIN 27

#define SENSOR_INTERVAL 100
#define OBSTACLE_DISTANCE 25.0f

float currentDistance = -1.0f;

// =====================================================
// 5. MOVEMENT COMMAND VARIABLES
// =====================================================

// Values received from Python.
// Linear:  -100 to +100
// Angular: -100 to +100

float targetLinear = 0.0f;
float targetAngular = 0.0f;

bool commandActive = false;

// =====================================================
// 6. TIMING VARIABLES
// =====================================================

#define COMMAND_TIMEOUT 500
#define WIFI_RETRY_INTERVAL 5000

unsigned long lastCommandMillis = 0;
unsigned long lastSensorMillis = 0;
unsigned long lastWiFiRetryMillis = 0;


// =====================================================
// 7. MOTOR INITIALIZATION
// =====================================================

void setupMotors() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Keep enable pins low during initialization.
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  // Attach ESP32 hardware PWM to enable pins.
  pwmLeftReady = ledcAttach(
    ENA,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  pwmRightReady = ledcAttach(
    ENB,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  if (!pwmLeftReady || !pwmRightReady) {

    Serial.println("ERROR: PWM initialization failed!");

    stopMotors();
    return;
  }

  stopMotors();

  Serial.println("Motors initialized.");
}


// =====================================================
// 8. INDIVIDUAL MOTOR CONTROL
// =====================================================

// speed:
// +100 = full forward
// -100 = full reverse
// 0    = stop

void setMotor(
  int pinA,
  int pinB,
  int enablePin,
  float speed,
  bool inverted,
  bool pwmReady
) {

  if (!pwmReady) {
    return;
  }

  speed = constrain(speed, -100.0f, 100.0f);

  if (inverted) {
    speed = -speed;
  }

  float magnitude = fabsf(speed);

  // Stop motor
  if (magnitude < 1.0f) {

    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);

    ledcWrite(enablePin, 0);

    return;
  }

  // Set motor direction
  if (speed > 0) {

    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);

  } else {

    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }

  // Convert percentage into PWM.
  // Apply minimum duty compensation.
  int pwm = MIN_MOTOR_PWM +
      (int)(
        (magnitude / 100.0f) *
        (PWM_MAX - MIN_MOTOR_PWM)
      );

  pwm = constrain(pwm, 0, PWM_MAX);

  ledcWrite(enablePin, pwm);
}


// =====================================================
// 9. DIFFERENTIAL DRIVE
// =====================================================

void driveMotors(
  float leftSpeed,
  float rightSpeed
) {

  if (!pwmLeftReady || !pwmRightReady) {
    stopMotors();
    return;
  }

  setMotor(
    IN1,
    IN2,
    ENA,
    leftSpeed,
    LEFT_MOTOR_INVERTED,
    pwmLeftReady
  );

  setMotor(
    IN3,
    IN4,
    ENB,
    rightSpeed,
    RIGHT_MOTOR_INVERTED,
    pwmRightReady
  );
}


// =====================================================
// 10. STOP MOTORS
// =====================================================

void stopMotors() {

  if (pwmLeftReady) {
    ledcWrite(ENA, 0);
  } else {
    digitalWrite(ENA, LOW);
  }

  if (pwmRightReady) {
    ledcWrite(ENB, 0);
  } else {
    digitalWrite(ENB, LOW);
  }

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


// =====================================================
// 11. DIFFERENTIAL DRIVE MIXING
// =====================================================

// Python provides linear and angular commands.
// ESP32 converts them into left and right motor speeds.

void executeVelocity(
  float linear,
  float angular
) {

  // Differential-drive equations
  float leftSpeed = linear - angular;
  float rightSpeed = linear + angular;

  // Normalize if either motor exceeds 100%.
  float maxSpeed = fmaxf(
    fabsf(leftSpeed),
    fabsf(rightSpeed)
  );

  if (maxSpeed > 100.0f) {

    leftSpeed =
        leftSpeed * 100.0f / maxSpeed;

    rightSpeed =
        rightSpeed * 100.0f / maxSpeed;
  }

  // Local obstacle safety.
  // Forward motion is prohibited when the sensor
  // reports an obstacle or an invalid reading.
  // Rotation in place and reverse remain possible.

  if (linear > 0.0f &&
      (currentDistance < 0.0f ||
       currentDistance <= OBSTACLE_DISTANCE)) {

    stopMotors();
    return;
  }

  driveMotors(leftSpeed, rightSpeed);
}


// =====================================================
// 12. ULTRASONIC DISTANCE MEASUREMENT
// =====================================================


float getDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    25000
  );

  // No echo within the measurement timeout.
  // Treat this as clear path / beyond sensor range.
  if (duration == 0) {
    return 400.0f;
  }

  // Calculate distance in centimetres.
  float distance = duration * 0.0343f / 2.0f;

  return distance;
}



// =====================================================
// 13. SEND SENSOR DATA TO PYTHON
// =====================================================

// Message format: DIST,42.5

void publishDistance() {

  if (!laptopKnown ||
      WiFi.status() != WL_CONNECTED) {

    return;
  }

  char message[32];

  snprintf(
    message,
    sizeof(message),
    "DIST,%.1f",
    currentDistance
  );

  udp.beginPacket(
    laptopIP,
    laptopPort
  );

  udp.write(
    (const uint8_t*)message,
    strlen(message)
  );

  udp.endPacket();
}


// =====================================================
// 14. RECEIVE COMMANDS FROM PYTHON
// =====================================================

// Message format: CMD,linear,angular

void receiveCommands() {

  int packetSize = udp.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  char packet[128];

  int len = udp.read(
    packet,
    sizeof(packet) - 1
  );

  if (len <= 0) {
    return;
  }

  packet[len] = '\0';

  float linear;
  float angular;

  if (sscanf(
        packet,
        "CMD,%f,%f",
        &linear,
        &angular
      ) == 2) {

    // Reject NaN and infinite values.
    if (!isfinite(linear) ||
        !isfinite(angular)) {
      return;
    }

    targetLinear = constrain(
      linear,
      -100.0f,
      100.0f
    );

    targetAngular = constrain(
      angular,
      -100.0f,
      100.0f
    );

    lastCommandMillis = millis();
    commandActive = true;

    // Learn the laptop's IP and UDP source port.
    laptopIP = udp.remoteIP();
    laptopPort = udp.remotePort();

    laptopKnown = true;

    Serial.printf(
      "Linear: %.1f | Angular: %.1f\n",
      targetLinear,
      targetAngular
    );
  }
}


// =====================================================
// 15. COMMAND TIMEOUT SAFETY
// =====================================================

void checkCommandTimeout() {

  if (commandActive &&
      millis() - lastCommandMillis > COMMAND_TIMEOUT) {

    targetLinear = 0.0f;
    targetAngular = 0.0f;

    commandActive = false;

    stopMotors();

    Serial.println("Command timeout. Motors stopped.");
  }
}


// =====================================================
// 16. WIFI INITIALIZATION
// =====================================================

void startUDP() {

  if (WiFi.status() == WL_CONNECTED &&
      !udpStarted) {

    udp.begin(UDP_PORT);

    udpStarted = true;

    Serial.print("UDP listening on port ");
    Serial.println(UDP_PORT);
  }
}


void connectWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  Serial.print("Connecting to WiFi");

  unsigned long startTime = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startTime < 10000) {

    delay(300);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println("WiFi connected!");

    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());

    startUDP();

  } else {

    Serial.println("WiFi connection failed.");
    stopMotors();
  }
}


// =====================================================
// 17. WIFI CONNECTION MONITOR
// =====================================================

void maintainWiFi() {

  if (WiFi.status() != WL_CONNECTED) {

    if (udpStarted) {
      udp.stop();
      udpStarted = false;
    }

    stopMotors();

    commandActive = false;

    targetLinear = 0.0f;
    targetAngular = 0.0f;

    if (millis() - lastWiFiRetryMillis >=
        WIFI_RETRY_INTERVAL) {

      lastWiFiRetryMillis = millis();

      Serial.println("Reconnecting to WiFi...");

      WiFi.reconnect();
    }

    return;
  }

  startUDP();
}


// =====================================================
// 18. SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println("ESP32 ROBOT CONTROLLER");

  // Initialize motors
  setupMotors();

  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  // Connect to WiFi
  connectWiFi();

  Serial.println("System ready.");
}


// =====================================================
// 19. MAIN LOOP
// =====================================================

void loop() {

  // Check WiFi connection
  maintainWiFi();

  // Receive incoming commands
  if (udpStarted) {
    receiveCommands();
  }

  // Check command timeout
  checkCommandTimeout();

  // Read sensor periodically
  if (millis() - lastSensorMillis >= SENSOR_INTERVAL) {

    lastSensorMillis = millis();

    currentDistance = getDistance();

    Serial.print("Distance: ");
    Serial.println(currentDistance);

    publishDistance();
  }

  // Execute latest movement command
  if (commandActive &&
      WiFi.status() == WL_CONNECTED) {

    executeVelocity(
      targetLinear,
      targetAngular
    );

  } else {

    stopMotors();
  }
}
