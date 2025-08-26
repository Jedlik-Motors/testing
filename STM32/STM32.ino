//=======================LEGEND=======================//

//Motor No. | Name                  | Timer No.| Wheel    | Encoder | 
//Motor 1   | Nema 23               |  2       | Steering |  A      |           
//Motor 2   | Nema 34 (T86, Green)  |  3       | Front    |  B      |           
//Motor 3   | Nema 34 (HSS86, Blue) |  5       | Rear     |  C      |           

//=======================LEGEND=======================//

#include <Arduino.h>
#include <HardwareTimer.h>

// Pin definitions
#define LED_PIN PC13
#define LED_ACTIVE_LOW true  // Most STM32F103 boards have active-low LED

// Serial communication settings
#define SERIAL_BAUD 9600
#define COMMAND_TIMEOUT 100  // milliseconds
#define MAX_COMMAND_LENGTH 32

// Command definitions
#define CMD_LED_ON "GLIDE_ON"
#define CMD_LED_OFF "CIRCLE_OFF"
#define RESP_LED_ON "OK_ON"
#define RESP_LED_OFF "OK_OFF"
#define RESP_ERROR "ERROR"

// Global variables
String inputBuffer = "";
unsigned long lastCommandTime = 0;
bool ledState = false;
bool commandInProgress = false;
bool invertMotorC = false;

long encA =0;
long encB =0;
long encC =0;

// Motor 1 pins
#define PUL1_PIN PB6
#define DIR1_PIN PC5
#define ENA1_PIN PA8

// Motor 2 pins
#define PUL2_PIN PC0
#define DIR2_PIN PC6
#define ENA2_PIN PA9

// Motor 3 pins
#define PUL3_PIN PC1
#define DIR3_PIN PB3
#define ENA3_PIN PB5

TIM_HandleTypeDef htim2; // Motor 1 Encoder
TIM_HandleTypeDef htim3; // Motor 2 Encoder
TIM_HandleTypeDef htim5; // Motor 3 Encoder


// Constants
const unsigned long TIMEOUT_MS = 500;
const int PULSE_WIDTH_US = 200;
const int PULSE_DELAY_US = 200;  // Adjust for motor speed

// State enum
enum State { IDLE, FOLLOWING_A, FOLLOWING_B, FOLLOWING_C };
State currentState = IDLE;

// Variables
long prevA = 0, prevB = 0, prevC = 0;
unsigned long lastChangeTime = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  
  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  setLEDState(false); // Start with LED OFF
  
  // Clear input buffer
  inputBuffer.reserve(MAX_COMMAND_LENGTH);
  
  // Wait for serial connection (optional, for debugging)
  delay(1000);
  
  // Send startup message
  Serial.println("STM32_READY");
  
  // Initialize timing
  lastCommandTime = millis();
  
  // Set pin modes
  pinMode(PUL1_PIN, OUTPUT); pinMode(DIR1_PIN, OUTPUT); pinMode(ENA1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT); pinMode(DIR2_PIN, OUTPUT); pinMode(ENA2_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT); pinMode(DIR3_PIN, OUTPUT); pinMode(ENA3_PIN, OUTPUT);
  
  // Initial: all enables HIGH (free)
  allEnablesHigh();

  //===== End of Motor pins setup ==========//


  // ==== TIM2 Encoder (NEMA23) ====
   TIM_Encoder_InitTypeDef sConfig2 = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig2.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig2.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig2.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig2.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig2.IC1Filter = 0;
  sConfig2.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig2.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig2.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig2.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig2) != HAL_OK) {
    Serial.println("Error: TIM2 Encoder Init Failed!");
    while(1);
  }

  // ==== TIM3 Encoder (Nema 34 - Green - Front) ====
  TIM_Encoder_InitTypeDef sConfig3 = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig3.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig3.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig3.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig3.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig3.IC1Filter = 0;
  sConfig3.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig3.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig3.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig3.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig3) != HAL_OK) {
    Serial.println("Error: TIM3 Encoder Init Failed!");
    while(1);
  }

   // ==== TIM3 Encoder (Nema 34 - Blue - Front) ====
  TIM_Encoder_InitTypeDef sConfig5 = {0};
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig5.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig5.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig5.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig5.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig5.IC1Filter = 0;
  sConfig5.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig5.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig5.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig5.IC2Filter = 0;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig5) != HAL_OK) {
    Serial.println("Error: TIM5 Encoder Init Failed!");
    while(1);
  }
  // Start all three encoders
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
    Serial.println("Error: TIM2 Encoder Start Failed!");
    while(1);
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
    Serial.println("Error: TIM3 Encoder Start Failed!");
    while(1);
  }
  if (HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK) {
    Serial.println("Error: TIM5 Encoder Start Failed!");
    while(1);
  }
  
  // Reset encoders if needed
  encA = 0;
  encB = 0;
  encC = 0;
}

void loop() {
    // Handle serial communication
  handleSerialCommunication();
  
  long currA = __HAL_TIM_GET_COUNTER(&htim2);
  long currB = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  long currC = __HAL_TIM_GET_COUNTER(&htim5);

  long deltaA = currA - prevA;
  long deltaB = currB - prevB;
  long deltaC = currC - prevC;
  
  prevA = currA;
  prevB = currB;
  prevC = currC;
  
  long absDeltaA = abs(deltaA);
  long absDeltaB = abs(deltaB);
  long absDeltaC = abs(deltaC);
  
  bool hasMovement = (deltaA != 0 || deltaB != 0 || deltaC != 0);
  
  if (currentState == IDLE) {
    if (hasMovement) {
      long maxDelta = max(max(absDeltaA, absDeltaB), absDeltaC);
      if (maxDelta == 0) return;
      
      if (maxDelta == absDeltaA) {
        currentState = FOLLOWING_A;
        digitalWrite(ENA2_PIN, LOW);
        digitalWrite(ENA3_PIN, LOW);
        driveMotorsSimultaneously('B', 'C', deltaA, invertMotorC ? -deltaA : deltaA);
      } else if (maxDelta == absDeltaB) {
        currentState = FOLLOWING_B;
        digitalWrite(ENA1_PIN, LOW);
        digitalWrite(ENA3_PIN, LOW);
        driveMotorsSimultaneously('A', 'C', deltaB, deltaB);
      } else {
        currentState = FOLLOWING_C;
        digitalWrite(ENA1_PIN, LOW);
        digitalWrite(ENA2_PIN, LOW);
        driveMotorsSimultaneously('A', 'B', deltaC, deltaC);
      }
      lastChangeTime = millis();
    }
  } else if (currentState == FOLLOWING_A) {
    if (deltaA != 0) {
      driveMotorsSimultaneously('B', 'C', deltaA, invertMotorC ? -deltaA : deltaA);
      lastChangeTime = millis();
    }
  } else if (currentState == FOLLOWING_B) {
    if (deltaB != 0) {
      driveMotorsSimultaneously('A', 'C', deltaB, deltaB);
      lastChangeTime = millis();
    }
  } else if (currentState == FOLLOWING_C) {
    if (deltaC != 0) {
      driveMotorsSimultaneously('A', 'B', deltaC, deltaC);
      lastChangeTime = millis();
    }
  }
  
  // Timeout check
  if (currentState != IDLE && (millis() - lastChangeTime > TIMEOUT_MS)) {
    allEnablesHigh();
    currentState = IDLE;
  }

  // Reset command timeout
  if (commandInProgress && (millis() - lastCommandTime) > COMMAND_TIMEOUT) {
    commandInProgress = false;
    inputBuffer = "";
  }
  
  // Small delay to prevent overwhelming the CPU
  delay(1);

}

// Drive two motors simultaneously
void driveMotorsSimultaneously(char motor1, char motor2, long steps1, long steps2) {
  if (steps1 == 0 && steps2 == 0) return;
  
  int pulsePin1, dirPin1, pulsePin2, dirPin2;
  
  // Assign pins for motor1
  if (motor1 == 'A') {
    pulsePin1 = PUL1_PIN;
    dirPin1 = DIR1_PIN;
  } else if (motor1 == 'B') {
    pulsePin1 = PUL2_PIN;
    dirPin1 = DIR2_PIN;
  } else {
    pulsePin1 = PUL3_PIN;
    dirPin1 = DIR3_PIN;
  }
  
  // Assign pins for motor2
  if (motor2 == 'A') {
    pulsePin2 = PUL1_PIN;
    dirPin2 = DIR1_PIN;
  } else if (motor2 == 'B') {
    pulsePin2 = PUL2_PIN;
    dirPin2 = DIR2_PIN;
  } else {
    pulsePin2 = PUL3_PIN;
    dirPin2 = DIR3_PIN;
  }
  
  // Set direction for both motors
  digitalWrite(dirPin1, (steps1 > 0) ? HIGH : LOW);
  digitalWrite(dirPin2, (steps2 > 0) ? HIGH : LOW);
  
  // Calculate maximum steps to synchronize
  long absSteps1 = abs(steps1);
  long absSteps2 = abs(steps2);
  long maxSteps = max(absSteps1, absSteps2);
  
  // Pulse both motors simultaneously
  for (long i = 0; i < maxSteps; i++) {
    if (i < absSteps1) {
      digitalWrite(pulsePin1, HIGH);
    }
    if (i < absSteps2) {
      digitalWrite(pulsePin2, HIGH);
    }
    delayMicroseconds(PULSE_WIDTH_US);
    if (i < absSteps1) {
      digitalWrite(pulsePin1, LOW);
    }
    if (i < absSteps2) {
      digitalWrite(pulsePin2, LOW);
    }
    delayMicroseconds(PULSE_DELAY_US);
  }
}

// Drive single motor (retained for completeness)
void driveMotor(char motor, long steps) {
  if (steps == 0) return;
  
  int pulsePin, dirPin;
  if (motor == 'A') {
    pulsePin = PUL1_PIN;
    dirPin = DIR1_PIN;
  } else if (motor == 'B') {
    pulsePin = PUL2_PIN;
    dirPin = DIR2_PIN;
  } else {
    pulsePin = PUL3_PIN;
    dirPin = DIR3_PIN;
  }
  
  digitalWrite(dirPin, (steps > 0) ? HIGH : LOW);
  long absSteps = abs(steps);
  for (long i = 0; i < absSteps; i++) {
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(PULSE_DELAY_US);
  }
}  

// Set all enables HIGH
void allEnablesHigh() {
  digitalWrite(ENA1_PIN, HIGH);
  digitalWrite(ENA2_PIN, HIGH);
  digitalWrite(ENA3_PIN, HIGH);
}


//==== Timer selection for encoder reading ================//

// ==== MSP Init for Encoder Pins ====
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    PA5     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_encoder->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA4     ------> TIM3_CH2
    PA6     ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_encoder->Instance==TIM5)
  {
    /* USER CODE BEGIN TIM5_MspInit 0 */

    /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PA0     ------> TIM5_CH1
    PC12     ------> TIM5_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM5_MspInit 1 */

    /* USER CODE END TIM5_MspInit 1 */
  }
}




void handleSerialCommunication() {
  // Check if data is available
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    // Handle newline characters (command terminator)
    if (inChar == '\n' || inChar == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
        commandInProgress = false;
      }
      return;
    }
    
    // Handle printable characters
    if (isPrintable(inChar) && inputBuffer.length() < MAX_COMMAND_LENGTH - 1) {
      inputBuffer += inChar;
      commandInProgress = true;
      lastCommandTime = millis();
    }
    
    // Handle buffer overflow
    if (inputBuffer.length() >= MAX_COMMAND_LENGTH - 1) {
      inputBuffer = "";
      commandInProgress = false;
      Serial.println(RESP_ERROR);
      return;
    }
  }
}

void processCommand(String command) {
  // Trim whitespace and convert to uppercase
  command.trim();
  command.toUpperCase();
  
  // Validate command length
  if (command.length() == 0 || command.length() > MAX_COMMAND_LENGTH) {
    Serial.println(RESP_ERROR);
    return;
  }
  
  // Process valid commands
  if (command.equals(CMD_LED_ON)) {
    setLEDState(true);
    invertMotorC = false; 
    Serial.println(RESP_LED_ON);
  }
  else if (command.equals(CMD_LED_OFF)) {
    setLEDState(false);
    invertMotorC = true;
    Serial.println(RESP_LED_OFF);
  }
  else {
    // Unknown command
    Serial.println(RESP_ERROR);
  }
}

void setLEDState(bool state) {
  ledState = state;
  
  // Apply LED polarity (most STM32F103 boards have active-low LED)
  if (LED_ACTIVE_LOW) {
    digitalWrite(LED_PIN, state ? LOW : HIGH);
  } else {
    digitalWrite(LED_PIN, state ? HIGH : LOW);
  }
}

bool getLEDState() {
  return ledState;
}

// Utility function to check if character is printable
bool isPrintable(char c) {
  return (c >= 32 && c <= 126);
}

// Optional: Heartbeat function for debugging
void sendHeartbeat() {
  static unsigned long lastHeartbeat = 0;
  const unsigned long heartbeatInterval = 30000; // 30 seconds
  
  if (millis() - lastHeartbeat >= heartbeatInterval) {
    Serial.print("HEARTBEAT:");
    Serial.print(ledState ? "ON" : "OFF");
    Serial.print(":");
    Serial.println(millis());
    lastHeartbeat = millis();
  }
}

// Optional: Command for getting current LED state
void handleStatusRequest() {
  Serial.print("STATUS:");
  Serial.print(ledState ? "ON" : "OFF");
  Serial.print(":");
  Serial.println(millis());
}

// Enhanced command processing with additional features
void processEnhancedCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  // Handle status request
  if (command.equals("STATUS")) {
    handleStatusRequest();
    return;
  }
  
  // Handle ping request
  if (command.equals("PING")) {
    Serial.println("PONG");
    return;
  }
  
  // Handle reset request
  if (command.equals("RESET")) {
    setLEDState(false);
    Serial.println("RESET_OK");
    return;
  }
  
  // Fall back to standard command processing
  processCommand(command);
}

// Error handling and recovery functions
void handleSerialError() {
  // Flush serial buffers
  while (Serial.available() > 0) {
    Serial.read();
  }
  
  // Clear input buffer
  inputBuffer = "";
  commandInProgress = false;
  
  // Send error response
  Serial.println(RESP_ERROR);
}

// Watchdog timer simulation (software-based)
void checkWatchdog() {
  static unsigned long lastActivity = millis();
  const unsigned long watchdogTimeout = 60000; // 60 seconds
  
  if (Serial.available() > 0) {
    lastActivity = millis();
  }
  
  // Reset LED to known state if no activity for too long
  if (millis() - lastActivity > watchdogTimeout) {
    setLEDState(false);
    lastActivity = millis();
  }
}

// Memory usage optimization
void optimizeMemory() {
  // Periodic cleanup of string objects
  static unsigned long lastCleanup = 0;
  const unsigned long cleanupInterval = 10000; // 10 seconds
  
  if (millis() - lastCleanup > cleanupInterval) {
    inputBuffer.trim(); // Remove any whitespace
    lastCleanup = millis();
  }
}