#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// -------- SENSOR PINNEN --------
#define IR_L 34
#define IR_R 32

// -------- MOTOR PINNEN (L298N) --------
#define enA 14
#define in1 27
#define in2 26
#define in3 13
#define in4 12
#define enB 4

// -------- PWM-snelheid (0–255) --------
#define MOTOR_SPEED 140
#define TURN_SPEED 160

// -------- LED PINNEN --------
#define LED_GREEN 23
#define LED_YELLOW 18
#define LED_RED 19

// -------- STARTKNOP --------
#define BUTTON_PIN 15
unsigned long lastButtonPress = 0;
unsigned long stopStartTime = 0;
const unsigned long DEBOUNCE_TIME = 200;
const unsigned long STOP_DURATION = 5000;
bool lastButtonState = HIGH;
bool buttonReleasedAfterStop = false;

// -------- ROBOTSTATUSSEN --------
enum RobotState { IDLE,
                  RUNNING,
                  STOPPED,
                  WAIT_FOR_REPRESS };
RobotState robotState = IDLE;
String currentStatus = "IDLE";

// -------- ULTRASONIC & SERVO PINS --------
#define triggerPin 2
#define echoPin 5
#define servoPin 25

// -------- DREMPELS --------
const int OBSTACLE_DIST = 20;             // cm voor
const int OBSTACLE_DIST_LATERAL = 30;     // cm sweep
const unsigned long BOTH_WAIT_MS = 3000;  // ms wacht als beide kanten geblokkeerd
int obstacleBlockCount = 0;               // telt aantal keer alle kanten geblokkeerd
const int OBSTACLE_BLOCK_LIMIT = 4;       // max keer voor actie

// -------- HORIZONTALE STREEP WAIT --------
const unsigned long HORIZ_WAIT_MS = 5000;    // ms wachten op streep
const unsigned long HORIZ_FORWARD_MS = 200;  // ms doorrijden na streep

// -------- BATTERIJ-MONITOR (ADC) --------
const int analogPin = 33;
const float r1 = 10000.0;
const float r2 = 5100.0;

// -------- Wi-Fi & MQTT --------
const char* ssid = "your_wifi_name";    
const char* wifi_pass = "your_wifi_password";   
const char* mqtt_server = "your_mqtt_ip";  
const int mqtt_port = 1883;
const char* mqtt_user = "linecar";
const char* mqtt_password = "wachtwoord";
const char* client_id = "ESP32_robot";
const char* topic_battery = "robot/battery";
const char* topic_status = "robot/status";
const char* topic_distance = "robot/distance";

WiFiClient espClient;
PubSubClient client(espClient);

// -------- TIMERS --------
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 1000;

// -------- STRAIGHT-DRIVE OVERRIDE --------
const unsigned long STRAIGHT_INTERVAL = 2500;  // ms rechtdoor voor U-turn 2000 bij low power,
const unsigned long TURN_180_DURATION = 1700;  // ms draai 180° //ongeveer 1700 bij max power, 2300 bij low power
unsigned long straightStart = 0;
bool goingStraight = false;

// -------- SERVO TRACKING --------
int servoAngle = 0;

// -------- STRUCT DEFINITIE VOOR SCAN --------
struct SweepResult {
  long distanceLeft;
  long distanceRight;
};

// -------- PROTOTYPES --------
void reconnectMQTT();
long Ultrasonic_read();
void avoidObstacle();
void servoPulse(int angle);
void servoToCenter();

// -------- SETUP --------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // IR-sensors
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);

  // Motor-pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // LED-pins
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // Ultrasoon en servo
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(servoPin, OUTPUT);

  // Startknop
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // WiFi
  WiFi.begin(ssid, wifi_pass);
  Serial.print("Verbinden met WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println(" WiFi connected");

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  // init servoAngle en zet servo in midden
  servoAngle = 0;
  servoToCenter();
}

// -------- MQTT RECONNECT --------
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Verbinden met MQTT…");
    if (client.connect(client_id, mqtt_user, mqtt_password)) {
      Serial.println("MQTT connected");
      client.subscribe("robot/remoteButton");
    } else {
      Serial.print(" gefaald, rc=");
      Serial.print(client.state());
      Serial.println(" opnieuw in 2 s");
      delay(2000);
    }
  }
}

// -------- MQTT CALLBACK --------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  if (String(topic) == "robot/remoteButton") {
    if (message == "BUTTON") {
      emulateButtonPress();
    }
  }
}

// -------- EMULEER DASHBOARD BUTTON --------
void emulateButtonPress() {
  unsigned long now = millis();

  // Reset straight-drive override, zodat je niet halverwege terugkomt in een lopende rechtdoor-periode
  goingStraight = false;
  straightStart = now;

  switch (robotState) {
    case IDLE:
      robotState = RUNNING;
      currentStatus = "RIJDEND";
      break;

    case RUNNING:
      robotState = STOPPED;
      stopStartTime = now;
      Stop();
      currentStatus = "GESTOPT";
      break;

    case WAIT_FOR_REPRESS:
      robotState = RUNNING;
      currentStatus = "RIJDEND";
      break;

    case STOPPED:
      break;
  }
}

// -------- PUBLISH DATA --------
void publishBatteryAndStatus() {
  const int NUM_SAMPLES = 10;
  long total = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += analogRead(analogPin);
    delay(5);
  }
  float avgRaw = total / float(NUM_SAMPLES);
  float vPin = (avgRaw / 4095.0) * 3.3;
  float battV = vPin * ((r1 + r2) / r2);

  const float MIN_V = 6.0;
  const float MAX_V = 7.2;
  int pct = constrain(int((battV - MIN_V) / (MAX_V - MIN_V) * 100), 0, 100);

  // LED's aansturen op basis van batterij
  digitalWrite(LED_GREEN, pct > 30);
  digitalWrite(LED_YELLOW, pct > 10 && pct <= 30);
  digitalWrite(LED_RED, pct <= 10);

  // Auto stoppen als batterij < 10% en hij rijdt
  if (pct <= 10 && robotState == RUNNING) {
    Stop();
    robotState = STOPPED;
    stopStartTime = millis();
    currentStatus = "LAGE BATTERIJ";
    Serial.println("Batterij te laag: robot gestopt");
  }

  // MQTT-publish
  char payload_batt[64];
  snprintf(payload_batt, sizeof(payload_batt), "{\"voltage\":%.2f,\"pct\":%d}", battV, pct);
  client.publish(topic_battery, payload_batt);
  client.publish(topic_status, currentStatus.c_str());

  Serial.print("Battery: ");
  Serial.println(payload_batt);
  Serial.print("Status: ");
  Serial.println(currentStatus);
}

// -------- PUBLISH AFSTAND --------
void publishDistance(long distance) {
  char payload_dist[64];
  snprintf(payload_dist, sizeof(payload_dist), "{\"distance\": %ld, \"unit\": \"cm\"}", distance);
  client.publish(topic_distance, payload_dist);
  Serial.print("Distance: ");
  Serial.println(payload_dist);
}

// -------- LOOP --------
void loop() {

  // MQTT upkeep
  if (!client.connected()) reconnectMQTT();
  client.loop();

  unsigned long now = millis();
  long dF = 999;
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishBatteryAndStatus();
    dF = Ultrasonic_read();
    publishDistance(dF);
  }

  // Knop + States
  bool currentButtonState = digitalRead(BUTTON_PIN);
  bool buttonPressed = currentButtonState == LOW && lastButtonState == HIGH && now - lastButtonPress > DEBOUNCE_TIME;
  lastButtonState = currentButtonState;

  switch (robotState) {
    case IDLE:
      if (buttonPressed) {
        servoToCenter();
        robotState = RUNNING;
        currentStatus = "RIJDEND";
      }
      break;

    case RUNNING:
      if (buttonPressed) {
        robotState = STOPPED;
        stopStartTime = now;
        Stop();
        currentStatus = "GESTOPT";
      }
      break;

    case STOPPED:
      Stop();
      if (now - stopStartTime >= STOP_DURATION) {
        robotState = WAIT_FOR_REPRESS;
        buttonReleasedAfterStop = false;
        currentStatus = "WACHT OP KNOP";
      }
      break;

    case WAIT_FOR_REPRESS:
      Stop();
      if (!buttonReleasedAfterStop && currentButtonState == HIGH) {
        buttonReleasedAfterStop = true;
      } else if (buttonReleasedAfterStop && buttonPressed) {
        robotState = RUNNING;
        currentStatus = "RIJDEND";
      }
      break;
  }

  delay(10);


  // Front-obstacle detectie
  Serial.printf("Ultrasonic front = %l cm\n", dF);
  if (robotState == RUNNING) {
    if (dF <= OBSTACLE_DIST) {
      avoidObstacle();
      return;
    }
  }

  // IR-lees & lijnvolg-logica
  if (robotState == RUNNING) {
    int L = digitalRead(IR_L);
    int R = digitalRead(IR_R);
    Serial.printf("IR L=%d R=%d\n", L, R);

    if (L == 0 && R == 0) {
      if (!goingStraight) {
        straightStart = now;
        goingStraight = true;
      }
      if (now - straightStart >= STRAIGHT_INTERVAL) {
        Stop();
        // 180° + terugrijden
        turnLeft();
        delay(TURN_180_DURATION);
        unsigned long driveStart = millis();
        while (millis() - driveStart < STRAIGHT_INTERVAL) {
          int L = digitalRead(IR_L);
          int R = digitalRead(IR_R);
          forward();

          // Als hij iets van de lijn detecteert → stop en herstel
          if (L == 1 || R == 1) {
            Serial.println("Lijn vroegtijdig gedetecteerd tijdens terugrit");
            Stop();
            delay(100);
            recoverParcours();
            break;
          }

          delay(10);
        }


        recoverParcours();
        Stop();
        delay(100);
        goingStraight = false;
      } else {
        forward();
      }
    } else {
      goingStraight = false;
      if (L == 1 && R == 0) {
        unsigned long start = now;

        turnLeft();
        delay(10);

      } else if (R == 1 && L == 0) {
        unsigned long start = now;

        turnRight();
        delay(10);

      } else if (L == 1 && R == 1) {
        // Horizontale streep
        Stop();
        servoToCenter();
        Serial.println("HORIZ-STREEP: wachten");
        delay(HORIZ_WAIT_MS);
        Serial.println("DOORRIJDEN 600ms");
        forward();
        delay(HORIZ_FORWARD_MS);
      }
    }

    delay(10);
  }
}

// -------- Motorfuncties --------
void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, MOTOR_SPEED);
  analogWrite(enB, MOTOR_SPEED);
  Serial.println("FORWARD");
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, TURN_SPEED);
  analogWrite(enB, TURN_SPEED);
  Serial.println("LEFT");
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, TURN_SPEED);
  analogWrite(enB, TURN_SPEED);
  Serial.println("RIGHT");
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  Serial.println("STOP");
}

// -------- ULTRASONIC --------
long Ultrasonic_read() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; 
}

SweepResult servoSweepScan() {
  SweepResult result;

  // Sweep naar rechts
  for (int a = 80; a <= 140; a += 5) {
    servoPulse(a);
    delay(30);
  }
  result.distanceRight = Ultrasonic_read();
  Serial.printf("  R = %l cm\n", result.distanceRight);
  delay(100);

  // Sweep naar links
  for (int a = 140; a >= 0; a -= 5) {
    servoPulse(a);
    delay(30);
  }
  result.distanceLeft = Ultrasonic_read();
  Serial.printf("  L = %l cm\n", result.distanceLeft);
  delay(100);

  return result;
}

// -------- OBSTACLE AVOID --------
void avoidObstacle() {
  Stop();
  servoToCenter();
  Serial.println("Obstacle! Scannen...");

  SweepResult scan = servoSweepScan();

  if (scan.distanceLeft <= OBSTACLE_DIST_LATERAL && scan.distanceRight <= OBSTACLE_DIST_LATERAL) {
    obstacleBlockCount++;
    Serial.printf("Beide zijden geblokkeerd (%d/%d)\n", obstacleBlockCount, OBSTACLE_BLOCK_LIMIT);

    if (obstacleBlockCount >= OBSTACLE_BLOCK_LIMIT) {
      Serial.println("Te vaak geblokkeerd → achteruit & U-turn");

      // Reset teller
      obstacleBlockCount = 0;

      servoToCenter();  // altijd eerst centreren

      // Achteruit
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);  // motor A achteruit
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);  // motor B achteruit
      analogWrite(enA, MOTOR_SPEED);
      analogWrite(enB, MOTOR_SPEED);
      delay(500);  // beetje achteruit
      Stop();

      // Nu draaien vóór recoverParcours
      turnLeft();
      delay(TURN_180_DURATION);
      Stop();

      // Herstel naar parcours
      recoverParcours();
      return;
    }

    // Anders: gewoon even wachten
    servoToCenter();
    delay(BOTH_WAIT_MS);
    return;
  } else {
    obstacleBlockCount = 0;  // reset als er wel ruimte is
  }
  servoToCenter();

  if (scan.distanceLeft > scan.distanceRight) {
    turnLeft();
    delay(500);
    forward();
    delay(850);
    turnRight();
    delay(500);
    forward();
    delay(900);
    turnRight();
    delay(650);
    forward();
    delay(200);
  } else {
    turnRight();
    delay(500);
    forward();
    delay(850);
    turnLeft();
    delay(500);
    forward();
    delay(900);
    turnLeft();
    delay(650);
    forward();
    delay(200);
  }
  goingStraight = false;
  Stop();
}

// -------- SERVO HELPERS --------
void servoPulse(int angle) {
  int us = map(angle, 0, 180, 500, 2400);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(us);
  digitalWrite(servoPin, LOW);
  delay(50);
  servoAngle = angle;
}
void servoToCenter() {
  while (servoAngle < 70) {
    servoPulse(servoAngle + 5);
  }
  while (servoAngle > 70) {
    servoPulse(servoAngle - 5);
  }
}

// -------- HERSTELROUTINE NA U-TURN --------
void recoverParcours() {
  Serial.println("Herstelroutine gestart");

  int L, R;

  // Zoek naar detectie van iets
  while (true) {
    L = digitalRead(IR_L);
    R = digitalRead(IR_R);
    if (L == 1 || R == 1) break;
    forward();
    delay(10);
  }

  Serial.printf("Gedetecteerd: L=%d R=%d\n", L, R);

  // Stop en rijd een klein beetje vooruit
  Stop();

  // Bepaal gedrag op basis van patroon
  if ((L == 1 && R == 0) || (L == 0 && R == 1)) {
    // Links of rechts op lijn: geen U-turn nodig
    Serial.println("Terug op lijn zonder U-turn");
    return;
  }

  if (L == 1 && R == 1) {
    // Beide detecteren zwart → mogelijk midden van lijn of kruising

    // Begin draaien (bijvoorbeeld linksom)
    while (true) {
      turnLeft();
      delay(10);
      L = digitalRead(IR_L);
      R = digitalRead(IR_R);
      if ((L == 1 && R == 0) || (L == 0 && R == 1)) break;
    }

    Stop();
    delay(100);
    Serial.println("Terug op lijn na U-turn");
  }
}
