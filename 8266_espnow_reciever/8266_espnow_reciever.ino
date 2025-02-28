#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>  // Add Servo library for ESC control

/*
Weapon ESC is the little bee - BLHeli_s 30A ESC Signal pin is connected to Esp8266 pin D0.

Drive motor (N20) is controlled by an ESC (not a true ESC) H-bridge DRV8833. D5&D6 connect to in 1&2 and D7&D8 connect to in 3&4

Robot is powered by a 30C 2S 450 mAh Lipo battery

Bot will be remote controlled via ESP-now. The protocol should send a packet at 20Hz. (every 50ms). If no packet is recieved for over 1 second the bot must go into a shut down mode (wheel motors to 0 and weapon spinner to halted with breaking)
*/

// Correct GPIO pin definitions for ESP8266
#define WeaponSignal 16  // D0 = GPIO 16
#define WheelLF 12      // D5 = GPIO 14
#define WheelLR 14      // D6 = GPIO 12
#define WheelRF 15      // D7 = GPIO 13
#define WheelRR 13      // D8 = GPIO 15

#define WHEEL_DIAMETER 30.0    // mm
#define AXLE_LENGTH 115.0      // mm

#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)

#define WHEEL_DIAMETER_MM 30.0
#define AXLE_LENGTH_MM 115.0
#define MOTOR_MAX_SPEED 1023  // Maximum speed for the motors

// Conversion factor for wheel speeds (linear and rotational components)
#define MM_TO_SPEED_FACTOR (MOTOR_MAX_SPEED / (WHEEL_DIAMETER_MM * PI))

// Create servo object to control the weapon ESC
Servo weaponESC;

// Struct to receive data 
struct Message{
  uint32_t key; //Code to manage access to device
  int16_t x1;  //Position vector - x desired (-1024 to 1024)
  int16_t x2;  //unused - always zero
  int16_t y1;  //Position vector - y desired (-1024 to 1024)
  int16_t y2;  //rotational vector - turns bot about center (-1024 to 1024)
  uint16_t trigger; //unused - always zero
  bool     bumpL;   //Toggle weapon halt (0 - bot is not allowed to spin weapon, weapon breaks are engaged.) (1 - bot is spinning weapon)
  bool     bumpR;   //Weapon active toggles (0 - counter rotation) (1 - normal rotation)
  bool     stickL;  // (button is unreliable. best to ignore it.)
  bool     stickR;  //inverts robot position
};
Message messageIn;
bool newMessage = 0;

// Safety timeout variables
unsigned long lastMessageTime = 0;
const unsigned long TIMEOUT_DURATION = 1000; // 1 second timeout

// Weapon control variables
bool weaponEnabled = false;
bool weaponDirection = true; // true = normal, false = counter
const int WEAPON_MIN_SIGNAL = 1000; // Minimum PWM signal (stopped)
const int WEAPON_MAX_SIGNAL = 2000; // Maximum PWM signal (full speed)

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  // Copy incoming data
  memcpy(&messageIn, incomingData, sizeof(messageIn));
  if(messageIn.key != 0x5C077BAD){
    Serial.println("key Rejected");
    return;
  }
  newMessage = 1;
  lastMessageTime = millis(); // Update last message timestamp
  digitalWrite(BUILTIN_LED, LOW);
  static int count = 0;
  // Print received message
  Serial.println("Received message: " + String(count));
  count++;
  Serial.println(messageIn.key);
  Serial.println(messageIn.x1);
  Serial.println(messageIn.x2);
  Serial.println(messageIn.y1);
  Serial.println(messageIn.y2);
  Serial.println(messageIn.trigger);
  Serial.println(messageIn.bumpL);
  Serial.println(messageIn.bumpR);
  Serial.println(messageIn.stickL);
  Serial.println(messageIn.stickR);
  Serial.println();
}

void setup() {
  // Initialize Serial first for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  Serial.println("Starting up...");
  
  // Configure pins
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(WheelLF, OUTPUT);
  pinMode(WheelLR, OUTPUT);
  pinMode(WheelRF, OUTPUT);
  pinMode(WheelRR, OUTPUT);
  
  // Initialize weapon ESC
  weaponESC.attach(WeaponSignal);
  weaponESC.writeMicroseconds(1000);  // Set to zero throttle
  
  // Initialize all drive outputs to 0
  analogWrite(WheelLF, 0);
  analogWrite(WheelLR, 0);
  analogWrite(WheelRF, 0);
  analogWrite(WheelRR, 0);
  
  digitalWrite(BUILTIN_LED, LOW);
  
  // Set PWM frequency for drive motors
  analogWriteFreq(14000);
  
 // Set device as WiFi station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else{
    Serial.println("esp now initialized");
  }

  // Set up receive callback
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void controlWeapon() {
  if (!weaponEnabled || !messageIn.bumpL) {
    weaponESC.writeMicroseconds(1000);  // Zero throttle with brake
    return;
  }
  
  // If weapon enabled and allowed to spin
  int weaponSpeed = messageIn.bumpR ? 2000 : 1100;  // Full throttle or zero
  weaponESC.writeMicroseconds(weaponSpeed);
}

void blink(){
  //change LED color
  if(digitalRead(BUILTIN_LED) == HIGH){
    digitalWrite(BUILTIN_LED, LOW);
  } else {
    digitalWrite(BUILTIN_LED, HIGH);
  }
}
// Function to control drive motors
void controlDriveMotors(int x, int y, int rotation) {
  // Normalize inputs to the -1023 to 1023 range
  x = constrain(x, -1023, 1023);
  y = constrain(y, -1023, 1023);
  rotation = constrain(rotation, -1023, 1023);

  float leftSpeed;
  float rightSpeed;

  if(!messageIn.stickL){
    //tank drive
    leftSpeed = y;
    rightSpeed = rotation;
  } else { 

    // Calculate wheel speeds
    leftSpeed = y + rotation * (AXLE_LENGTH_MM / 2.0) * MM_TO_SPEED_FACTOR;
    rightSpeed = y - rotation * (AXLE_LENGTH_MM / 2.0) * MM_TO_SPEED_FACTOR;

    // Add strafe component
    leftSpeed += x;
    rightSpeed -= x;

    // Normalize speeds to the motor maximum speed
    float maxMagnitude = max(abs(leftSpeed), abs(rightSpeed));
    if (maxMagnitude > MOTOR_MAX_SPEED) {
      leftSpeed = (leftSpeed / maxMagnitude) * MOTOR_MAX_SPEED;
      rightSpeed = (rightSpeed / maxMagnitude) * MOTOR_MAX_SPEED;
    }

  }
  // Invert controls if stickR is pressed
  if (messageIn.stickR) {
    leftSpeed = -leftSpeed;
    rightSpeed = -rightSpeed;
  }

  // Set motor speeds for the left side
  if (leftSpeed >= 0) {
    analogWrite(WheelLF, leftSpeed);
    analogWrite(WheelLR, 0);
  } else {
    analogWrite(WheelLF, 0);
    analogWrite(WheelLR, -leftSpeed);
  }

  // Set motor speeds for the right side
  if (rightSpeed >= 0) {
    analogWrite(WheelRF, rightSpeed);
    analogWrite(WheelRR, 0);
  } else {
    analogWrite(WheelRF, 0);
    analogWrite(WheelRR, -rightSpeed);
  }
}

void shutdownSystems() {
  // Stop all motors
  analogWrite(WheelLF, 0);
  analogWrite(WheelLR, 0);
  analogWrite(WheelRF, 0);
  analogWrite(WheelRR, 0);
  
  // Stop weapon with brake
  analogWrite(WeaponSignal, WEAPON_MIN_SIGNAL);
  
  weaponEnabled = false;
}

void loop() {
  // Check for timeout
  if (millis() - lastMessageTime > TIMEOUT_DURATION) {
    shutdownSystems();
    digitalWrite(BUILTIN_LED, HIGH);//blink(); // Visual indication of timeout
    return;
  }
  
  if (newMessage == 1) {
    newMessage = 0;
    
    // Update weapon state
    weaponEnabled = messageIn.bumpL;
    weaponDirection = messageIn.bumpR;
    
    // Control systems
    controlWeapon();
    controlDriveMotors(messageIn.x1, messageIn.y1, messageIn.y2);
  }
}
