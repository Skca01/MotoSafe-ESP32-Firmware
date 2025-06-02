/**
 * Motorcycle Security System
 * 
 * This system provides comprehensive security and remote control features for motorcycles:
 * - Movement detection using an MPU6050 accelerometer
 * - GPS tracking and speed monitoring
 * - GSM communication for SMS alerts and call functionality
 * - Remote control via SMS commands
 * - Voltage-based tampering detection
 * - Motorcycle kill switch and horn control
 * 
 * Hardware components:
 * - ESP32 microcontroller
 * - MPU6050 accelerometer
 * - NEO-8M GPS module
 * - SIM800L GSM module
 * - IRLZ44N MOSFET for power control
 * - Relays for kill switch and horn control
 * - Voltage divider for tampering detection
 */

#include <WiFi.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Preferences.h>

//==============================================================
// Pin Definitions
//==============================================================
const int MOSFET_PIN = 27;      // IRLZ44N MOSFET gate pin for power control
const int RELAY_KILL_PIN = 4;   // Relay for motorcycle kill switch
const int RELAY_HORN_PIN = 19;  // Relay for horn control
const int VOLTAGE_SENSOR = 33;  // Analog pin for voltage divider (tampering detection)
const int MPU_SDA = 21;         // Default I2C SDA pin for ESP32
const int MPU_SCL = 22;         // Default I2C SCL pin for ESP32
const int SIM800L_RX = 14;      // SIM800L RX connected to D14
const int SIM800L_TX = 13;      // SIM800L TX connected to D13
const int GPS_RX = 17;          // NEO-8M RX connected to GPIO 17
const int GPS_TX = 16;          // NEO-8M TX connected to GPIO 16

//==============================================================
// MPU6050 Constants
//==============================================================
const int MPU_ADDR = 0x68;      // MPU6050 I2C address
const int ACCEL_XOUT_H = 0x3B;  // Register address for accelerometer data
const int PWR_MGMT_1 = 0x6B;    // Power management register

//==============================================================
// Hardware Serial Configuration
//==============================================================
HardwareSerial simSerial(1);    // UART1 for SIM800L
HardwareSerial gpsSerial(2);    // UART2 for GPS

// Preferences for persistent storage across reboots
Preferences preferences;

// GPS parser object
TinyGPSPlus gps;

//==============================================================
// System Status Variables
//==============================================================
bool mpu_ok = false;            // MPU6050 initialization status
bool sim_ok = false;            // SIM800L initialization status
bool gps_ok = false;            // GPS initialization status
bool trackingEnabled = false;   // GPS tracking state
bool alertsEnabled = true;      // Alert notification state
bool motorcycleEnabled = true;  // Motorcycle enabled state (true = running)
bool waitingForAcknowledge = false; // Flag for alert acknowledgment
bool hornActivated = false;     // Horn status
bool systemSleepMode = false;   // Low power sleep mode status
bool disablePending = false;    // Flag for pending motorcycle disable request
bool callInProgress = false;    // Flag to track if a call is active

//==============================================================
// Timing Variables
//==============================================================
unsigned long lastCallAttempt = 0;  // Time of last call attempt
const int CALL_RETRY_INTERVAL = 60000;  // Wait 1 minute between call attempts
const int CALL_DURATION = 20000;  // Call duration in milliseconds (20 seconds)
unsigned long callStartTime = 0;  // Timestamp when call was started

// Owner phone number - stored in preferences
String OWNER_PHONE = "";  

//==============================================================
// MPU6050 Movement Detection Variables
//==============================================================
float accelThreshold = 0.3;  // Threshold for detecting movement (in g)
float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0; // Previous acceleration values
unsigned long lastMovementCheck = 0;  // Timestamp of last movement check
const int MOVEMENT_CHECK_INTERVAL = 500; // Check for movement every 500ms
bool movementDetected = false;  // Flag for movement detection state

//==============================================================
// GPS Tracking Variables
//==============================================================
unsigned long lastTrackingUpdate = 0;  // Timestamp of last tracking update
const int TRACKING_INTERVAL = 10000; // Send location every 10 seconds

//==============================================================
// Voltage Tampering Variables
//==============================================================
const int VOLTAGE_THRESHOLD = 1800; // Threshold for voltage tamper detection (analog value)
unsigned long lastVoltageCheck = 0;  // Timestamp of last voltage check
const int VOLTAGE_CHECK_INTERVAL = 1000; // Check voltage every second
bool tamperingDetected = false;  // Flag for tampering detection state
unsigned long lastTamperAlert = 0;  // Time of last tamper alert
const int TAMPER_ALERT_COOLDOWN = 30000;  // 30 seconds cooldown between tamper alerts

//==============================================================
// Alert Acknowledgment Variables
//==============================================================
unsigned long acknowledgeStartTime = 0;  // Start time for acknowledgment timer
const int ACKNOWLEDGE_TIMEOUT = 30000; // 30 seconds timeout before horn activation

//==============================================================
// Horn Pattern Variables
//==============================================================
unsigned long lastHornToggle = 0;  // Timestamp of last horn state change
const int HORN_ON_DURATION = 5000;   // Horn on for 5 seconds
const int HORN_OFF_DURATION = 1000;  // Horn off for 1 second
bool hornState = false;              // Current state of the horn (true = on)

//==============================================================
// SMS Processing Variables
//==============================================================
String receivedSMS = "";  // Buffer for received SMS messages
unsigned long lastSMSCheck = 0;  // Timestamp of last SMS check
const int SMS_CHECK_INTERVAL = 1000; // Check for SMS every 1 second

//==============================================================
// Speed Check Variables
//==============================================================
unsigned long lastSpeedCheck = 0;  // Timestamp of last speed check
const int SPEED_CHECK_INTERVAL = 1000; // Check speed every second
const float SAFE_DISABLE_SPEED = 13.0; // Safe speed threshold in km/h for disabling motorcycle

//==============================================================
// Function Prototypes
//==============================================================
String sendATCommand(String command, int timeout);
void checkMPUMovement();
void checkVoltage();
void sendAlert(String alertType);
void sendTrackingData();
void processCommands(String message);
void activateHorn(bool activate);
void updateHornPattern();
void controlMotorcycle(bool enable);
void setupMPU6050();
void savePhoneNumber(String phoneNumber);
String loadPhoneNumber();
void enableSleepMode();
void wakeFromSleepMode();
void sendSMS(String message);
void checkSMS();
bool initializeSIM800L();
void placeCall();
void checkCallStatus();

/**
 * Save owner phone number to persistent storage
 * 
 * @param phoneNumber The phone number to save
 */
void savePhoneNumber(String phoneNumber) {
  preferences.begin("security", false);  // Open preferences in RW mode
  preferences.putString("ownerPhone", phoneNumber);  // Store the phone number
  preferences.end();  // Close preferences
}

/**
 * Load owner phone number from persistent storage
 * 
 * @return The stored phone number or empty string if not found
 */
String loadPhoneNumber() {
  preferences.begin("security", true);  // Open preferences in read-only mode
  String storedNumber = preferences.getString("ownerPhone", "");  // Get stored number or empty string
  preferences.end();  // Close preferences
  return storedNumber;
}

/**
 * Enable sleep mode to conserve power
 * - Turns off MOSFET to disable peripheral power
 * - Disables motorcycle and critical systems
 * - Stores sleep state in preferences
 */
void enableSleepMode() {
  // Check if already in sleep mode
  if (systemSleepMode) return;

  // Send sleep mode notification
  sendSMS("SYSTEM SLEEP MODE\n- MOSFET powered off\n- Critical systems suspended\n- Low power consumption activated");
  
  // Set sleep mode flag
  systemSleepMode = true;
  
  // Save sleep mode state in Preferences
  preferences.begin("security", false);
  preferences.putBool("sleepMode", true);
  preferences.end();
  
  // Turn off MOSFET to cut power to peripherals
  digitalWrite(MOSFET_PIN, LOW);
  
  // Disable motorcycle and other critical systems
  digitalWrite(RELAY_KILL_PIN, LOW);  // Kill the motorcycle
  digitalWrite(RELAY_HORN_PIN, HIGH); // Ensure horn is off
  
  // Disable tracking and alerts
  trackingEnabled = false;
  alertsEnabled = false;
}

/**
 * Wake system from sleep mode
 * - Restores power to peripherals
 * - Reinitializes system components
 * - Clears sleep state in preferences
 */
void wakeFromSleepMode() {
  // Check if system was in sleep mode
  preferences.begin("security", true);
  bool wasInSleepMode = preferences.getBool("sleepMode", false);
  preferences.end();
  
  if (!wasInSleepMode) return;
  
  // Turn on MOSFET to restore power to peripherals
  digitalWrite(MOSFET_PIN, HIGH);
  
  // Reset sleep mode flag
  systemSleepMode = false;
  
  // Clear sleep mode state in Preferences
  preferences.begin("security", false);
  preferences.putBool("sleepMode", false);
  preferences.end();
  
  // Send wake-up notification
  sendSMS("SYSTEM AWAKENED\n- MOSFET powered on\n- All systems reinitializing\n- Full operational mode restored");
  
  // Reinitialize all system components
  setup();
}

/**
 * System initialization function
 * - Sets up all hardware components
 * - Initializes communication modules
 * - Restores system state
 */
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("Initializing motorcycle security system...");
  
  // Load stored phone number from persistent storage
  OWNER_PHONE = loadPhoneNumber();
  
  // Check if system was in sleep mode before restart
  preferences.begin("security", true);
  bool wasInSleepMode = preferences.getBool("sleepMode", false);
  preferences.end();
  
  // Initialize control pins
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(RELAY_KILL_PIN, OUTPUT);
  pinMode(RELAY_HORN_PIN, OUTPUT);
  pinMode(VOLTAGE_SENSOR, INPUT);
  
  // If recovering from sleep mode, temporarily keep MOSFET off
  if (wasInSleepMode) {
    digitalWrite(MOSFET_PIN, LOW);
    return;  // Exit setup to allow wake command processing
  }
  
  // Set initial power and relay states
  digitalWrite(MOSFET_PIN, HIGH);  // Turn on MOSFET to enable power
  digitalWrite(RELAY_KILL_PIN, HIGH);  // Default state: motorcycle enabled (NC relay)
  digitalWrite(RELAY_HORN_PIN, HIGH); // Default state: horn off (NO relay)
  
  Serial.println("Power and relay pins initialized");
  
  // Give devices time to power up
  delay(2000);
  
  // Implement retry mechanism for initialization (up to 3 attempts)
  int initRetries = 3;
  while (initRetries > 0) {
    // Reset initialization flags
    mpu_ok = false;
    sim_ok = false;
    gps_ok = false;
    
    // Initialize I2C for MPU6050 accelerometer
    Wire.begin(MPU_SDA, MPU_SCL);
    setupMPU6050();
    
    // Initialize SIM800L GSM module
    simSerial.begin(9600, SERIAL_8N1, SIM800L_TX, SIM800L_RX);
    sim_ok = initializeSIM800L();
    
    // Initialize GPS module
    gpsSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
    
    // Check for GPS data (try for 5 seconds)
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
      while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
          gps_ok = true;
          Serial.println("GPS initialized successfully");
          break;
        }
      }
      if (gps_ok) break;
      delay(10);
    }
    
    // If all critical components are initialized, break the retry loop
    if (mpu_ok && sim_ok) {
      break;
    }
    
    initRetries--;
    Serial.println("Initialization failed. Retrying...");
    delay(1000);
  }
  
  // Send detailed initialization status via SMS
  if (sim_ok) {
    String message = "SYSTEM INITIALIZED\n";
    message += "MPU6050: " + String(mpu_ok ? "OPERATIONAL" : "FAULT") + "\n";
    message += "SIM800L: " + String(sim_ok ? "CONNECTED" : "DISCONNECTED") + "\n";
    message += "GPS: " + String(gps_ok ? "ACTIVE" : "NO FIX") + "\n";
    
    // If GPS is working, include initial location
    if (gps_ok && gps.location.isValid()) {
      char locationStr[100];
      sprintf(locationStr, "Location: %.6f, %.6f", 
              gps.location.lat(), gps.location.lng());
      message += locationStr;
    }
    
    sendSMS(message);
  } else {
    Serial.println("Unable to send initialization SMS. SIM800L failed.");
  }
  
  Serial.println("Setup complete. System running.");
}

/**
 * Main program loop
 * - Processes GPS data
 * - Checks sensors and system status
 * - Handles alerts and command processing
 * - Manages tracking and communication
 */
void loop() {
  // First, check if we need to wake from sleep mode
  if (systemSleepMode) {
    // In sleep mode, only process SMS to potentially wake up
    if (millis() - lastSMSCheck > SMS_CHECK_INTERVAL) {
      lastSMSCheck = millis();
      checkSMS();  // Check for SMS commands like "wake"
    }
    return;  // Skip all other processing while in sleep mode
  }
  
  // Process incoming GPS data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (gps.encode(c) && !gps_ok) {
      gps_ok = true;
      Serial.println("GPS connection established");
    }
  }
  
  // Check ongoing call status
  if (callInProgress) {
    checkCallStatus();
  }
  
  // Check for incoming SMS commands
  if (millis() - lastSMSCheck > SMS_CHECK_INTERVAL) {
    lastSMSCheck = millis();
    checkSMS();
  }
  
  // Check for pending disable request (safety feature)
  if (disablePending && gps_ok && millis() - lastSpeedCheck > SPEED_CHECK_INTERVAL) {
    lastSpeedCheck = millis();
    
    if (gps.location.isValid() && gps.speed.isValid()) {
      float currentSpeed = gps.speed.kmph();
      
      // If speed has dropped below threshold, now safe to disable the motorcycle
      if (currentSpeed <= SAFE_DISABLE_SPEED) {
        motorcycleEnabled = false;
        digitalWrite(RELAY_KILL_PIN, LOW);  // Disable the motorcycle
        disablePending = false; // Clear the pending flag
        sendSMS("Speed now below threshold (" + String(currentSpeed, 1) + 
                " km/h). Motorcycle has been automatically disabled.");
      }
    }
  }
  
  // Check for unauthorized movement if alerts are enabled
  if (alertsEnabled && mpu_ok && millis() - lastMovementCheck > MOVEMENT_CHECK_INTERVAL) {
    lastMovementCheck = millis();
    checkMPUMovement();
  }
  
  // Check voltage for tampering detection if alerts are enabled
  if (alertsEnabled && millis() - lastVoltageCheck > VOLTAGE_CHECK_INTERVAL) {
    lastVoltageCheck = millis();
    checkVoltage();
  }
  
  // Update horn pattern if horn is activated
  if (hornActivated) {
    updateHornPattern();
  }
  
  // Send tracking data if enabled
  if (trackingEnabled && gps_ok && millis() - lastTrackingUpdate > TRACKING_INTERVAL) {
    lastTrackingUpdate = millis();
    sendTrackingData();
  }
  
  // Check for acknowledge timeout (activate horn if no acknowledgment received)
  if (waitingForAcknowledge && millis() - acknowledgeStartTime > ACKNOWLEDGE_TIMEOUT) {
    waitingForAcknowledge = false;
    activateHorn(true);  // Activate horn after timeout
    sendSMS("ALERT: No acknowledge received. Horn activated!");
  }
}

/**
 * Places a phone call to the registered owner number
 * - Used during security alerts
 * - Manages call duration and retry intervals
 */
void placeCall() {
  if (!sim_ok || OWNER_PHONE.length() == 0) return;
  
  Serial.println("Placing call to owner: " + OWNER_PHONE);
  
  // Command to dial the number (ATD = Dial, semicolon = voice call)
  String response = sendATCommand("ATD" + OWNER_PHONE + ";", 5000);
  
  if (response.indexOf("OK") != -1) {
    callInProgress = true;
    callStartTime = millis();
    lastCallAttempt = millis();
    Serial.println("Call initiated successfully");
  } else {
    Serial.println("Failed to initiate call");
    lastCallAttempt = millis();  // Still update the time to avoid immediate retry
  }
}

/**
 * Checks the status of an ongoing call
 * - Monitors call duration
 * - Detects if call was answered and then hung up
 * - Handles call termination
 */
void checkCallStatus() {
  if (!callInProgress) return;
  
  // Check if call duration exceeded maximum time
  if (millis() - callStartTime > CALL_DURATION) {
    // Hang up the call (ATH = Hang up)
    sendATCommand("ATH", 1000);
    callInProgress = false;
    Serial.println("Call ended due to timeout");
    return;
  }
  
  // Check call status (AT+CLCC = List Current Calls)
  String response = sendATCommand("AT+CLCC", 1000);
  
  // If no call in list or call status indicates it's disconnected
  // ",0,6" in response indicates call state is "disconnect"
  if (response.indexOf("+CLCC:") == -1 || response.indexOf(",0,6") != -1) {
    callInProgress = false;
    Serial.println("Call ended");
  }
}

/**
 * Updates horn pattern for alarm function
 * - Implements 5 seconds on, 1 second off pattern
 * - Attempts to place call when horn is activated
 */
void updateHornPattern() {
  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - lastHornToggle;
  
  // If horn is ON and time to turn OFF
  if (hornState && timeElapsed > HORN_ON_DURATION) {
    hornState = false;
    lastHornToggle = currentTime;
    digitalWrite(RELAY_HORN_PIN, HIGH);  // Turn OFF (HIGH deactivates the horn)
    Serial.println("Horn OFF");
  } 
  // If horn is OFF and time to turn ON
  else if (!hornState && timeElapsed > HORN_OFF_DURATION) {
    hornState = true;
    lastHornToggle = currentTime;
    digitalWrite(RELAY_HORN_PIN, LOW);   // Turn ON (LOW activates the horn)
    Serial.println("Horn ON");
    
    // Attempt to place a call if horn is activated and not already calling
    if (hornActivated && !callInProgress && (millis() - lastCallAttempt > CALL_RETRY_INTERVAL)) {
      placeCall();
    }
  }
}

/**
 * Initializes the SIM800L GSM module
 * - Sets up SMS text mode
 * - Configures character set and call notifications
 * - Clears SMS memory
 * 
 * @return true if initialization successful, false otherwise
 */
bool initializeSIM800L() {
  // Clear the serial buffer
  while (simSerial.available()) {
    simSerial.read();
  }
  
  // Test AT command (basic communication test)
  String response = sendATCommand("AT", 1000);
  if (response.indexOf("OK") == -1) {
    Serial.println("SIM800L not responding");
    return false;
  }
  
  // Set SMS text mode (AT+CMGF=1 = Text mode, AT+CMGF=0 = PDU mode)
  response = sendATCommand("AT+CMGF=1", 1000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set SMS text mode");
    return false;
  }
  
  // Set character set to GSM (standard for SMS)
  response = sendATCommand("AT+CSCS=\"GSM\"", 1000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set character set");
  }
  
  // Enable call status notifications (CLIP = Calling Line Identification Presentation)
  response = sendATCommand("AT+CLIP=1", 1000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to enable CLIP");
  }
  
  // Delete all SMS to free up memory
  // AT+CMGD=1,4 = Delete all messages (1=index, 4=delete all)
  response = sendATCommand("AT+CMGD=1,4", 1000);
  
  Serial.println("SIM800L initialized successfully");
  return true;
}

/**
 * Sends AT command to SIM800L with timeout
 * 
 * @param command AT command to send
 * @param timeout Maximum time to wait for response in milliseconds
 * @return Response string from the module
 */
String sendATCommand(String command, int timeout) {
  simSerial.println(command);
  
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < timeout) {
    if (simSerial.available()) {
      char c = simSerial.read();
      response += c;
      
      // If we got OK or ERROR, we can exit early
      if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
        delay(50); // Small delay to get any remaining data
        while (simSerial.available()) {
          response += (char)simSerial.read();
        }
        break;
      }
    }
    delay(10);
  }
  
  return response;
}

/**
 * Initializes the MPU6050 accelerometer
 * - Sets up I2C communication
 * - Configures power management and sensitivity
 * - Calibrates initial position
 */
void setupMPU6050() {
  // Check if MPU6050 is responding
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("MPU6050 found");
    
    // Wake up the MPU6050 (clear sleep bit)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);  // PWR_MGMT_1 register
    Wire.write(0);           // Set to zero to wake up
    error = Wire.endTransmission();
    
    if (error == 0) {
      // Set accelerometer sensitivity to ±2g (most sensitive)
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x1C);  // ACCEL_CONFIG register
      Wire.write(0x00);  // 2g full scale range (0x00 = 2g, 0x08 = 4g, 0x10 = 8g, 0x18 = 16g)
      Wire.endTransmission();
      
      // Read initial acceleration values for calibration
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(ACCEL_XOUT_H);  // Start reading from accelerometer data registers
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, true);  // Request 6 bytes (2 bytes x 3 axes)
      
      // Combine high and low bytes to form 16-bit values
      int16_t rawX = Wire.read() << 8 | Wire.read();
      int16_t rawY = Wire.read() << 8 | Wire.read();
      int16_t rawZ = Wire.read() << 8 | Wire.read();
      
      // Convert raw values to g (±2g range, 16384 = 1g)
      prevAccelX = rawX / 16384.0;
      prevAccelY = rawY / 16384.0;
      prevAccelZ = rawZ / 16384.0;
      
      mpu_ok = true;
      Serial.println("MPU6050 initialized successfully");
    } else {
      Serial.println("Error initializing MPU6050");
    }
  } else {
    Serial.println("MPU6050 not found, check wiring!");
  }
}

/**
 * Checks for unauthorized movement using MPU6050
 * - Reads acceleration values
 * - Calculates change from previous readings
 * - Triggers alert if movement exceeds threshold
 */
void checkMPUMovement() {
  if (!mpu_ok) return;
  
  // Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  // Request 6 bytes (2 bytes x 3 axes)
  
  // Combine high and low bytes for each axis
  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();
  
  // Convert to g values (±2g range, 16384 = 1g)
  float accelX = rawX / 16384.0;
  float accelY = rawY / 16384.0;
  float accelZ = rawZ / 16384.0;
  
  // Calculate changes in acceleration (delta)
  float deltaX = abs(accelX - prevAccelX);
  float deltaY = abs(accelY - prevAccelY);
  float deltaZ = abs(accelZ - prevAccelZ);
  
  // Check if movement exceeds threshold
  if (deltaX > accelThreshold || deltaY > accelThreshold || deltaZ > accelThreshold) {
    if (!movementDetected) {
      movementDetected = true;
      sendAlert("MOVEMENT");  // Send movement alert
      
      // Start acknowledge timeout
      acknowledgeStartTime = millis();
      waitingForAcknowledge = true;
    }
  } else {
    // Reset movement detection after period of stability
    movementDetected = false;
  }
  
  // Store current values for next comparison
  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;
}

/**
 * Checks for voltage tampering
 * - Reads voltage sensor value
 * - Compares with threshold
 * - Triggers alert with cooldown period
 */
void checkVoltage() {
  int voltageValue = analogRead(VOLTAGE_SENSOR);
  
  // Voltage higher than threshold indicates tampering
  if (voltageValue > VOLTAGE_THRESHOLD) {
    if (!tamperingDetected) {
      tamperingDetected = true;
      
      // Only send alert if cooldown period has passed
      if (millis() - lastTamperAlert > TAMPER_ALERT_COOLDOWN) {
        lastTamperAlert = millis();
        sendAlert("TAMPERING");
        
        // Start acknowledge timeout
        acknowledgeStartTime = millis();
        waitingForAcknowledge = true;
      }
    }
  } else {
    tamperingDetected = false;
  }
}

/**
 * Sends GPS tracking data via SMS
 * - Formats GPS coordinates and speed
 * - Includes satellite count for signal quality indication
 */
void sendTrackingData() {
  if (!gps.location.isValid()) {
    Serial.println("GPS location not valid, can't send tracking data");
    return;
  }
  
  // Format tracking data with GPS coordinates and speed
  char message[160];
  sprintf(message, "TRACKING: Lat=%f, Lng=%f, Speed=%fkm/h, Sats=%d",
          gps.location.lat(), gps.location.lng(),
          gps.speed.kmph(), gps.satellites.value());
  
  sendSMS(String(message));
}

/**
 * Sends security alert via SMS
 * - Formats alert based on type (movement, tampering, ignition)
 * - Includes GPS location if available
 * - Adds instructions for acknowledgment
 * 
 * @param alertType Type of alert to send (MOVEMENT, TAMPERING, IGNITION)
 */
void sendAlert(String alertType) {
  if (!sim_ok) return;
  
  String message = "ALERT: ";
  
  // Create message based on alert type
  if (alertType == "MOVEMENT") {
    message += "Unauthorized movement detected!";
  } else if (alertType == "TAMPERING") {
    message += "Electrical tampering detected!";
  } else if (alertType == "IGNITION") {
    message += "Unauthorized ignition attempt!";
  }
  
  // Add location information if available
  if (gps.location.isValid()) {
    char locationStr[80];
    sprintf(locationStr, " Location: %.6f, %.6f", 
            gps.location.lat(), gps.location.lng());
    message += locationStr;
  }
  
  // Add instructions for acknowledgment
  message += " Reply 'acknowledge' to stop horn, or wait 30 seconds for auto-horn. A call will also be placed.";
  
  sendSMS(message);
  
  // Schedule a call attempt
  lastCallAttempt = 0;  // Set to 0 to trigger a call immediately in the next cycle
}

/**
 * Sends SMS message to owner's phone number
 *  - Checks if SIM module is operational and phone number exists
 *  - Uses AT commands to send the SMS
 *  @param message Text content to send
 */
void sendSMS(String message) {
  if (!sim_ok || OWNER_PHONE.length() == 0) return;
  
  Serial.println("Sending SMS: " + message);
  
  sendATCommand("AT+CMGS=\"" + OWNER_PHONE + "\"", 1000);
  simSerial.print(message);
  simSerial.write(26); // Ctrl+Z to send message
  
  delay(500);
}

/**
 * Checks for incoming SMS messages
 *  - Polls SIM module for unread messages
 *  - Extracts message content when found
 *  - Processes commands from the message
 *  - Deletes read messages to free memory
 */
void checkSMS() {
  if (!sim_ok) return;
  
  // Check for unread messages
  String response = sendATCommand("AT+CMGL=\"REC UNREAD\"", 5000);
  
  if (response.indexOf("+CMGL:") != -1) {
    // Extract SMS content
    int msgStart = response.indexOf("\r\n", response.indexOf("+CMGL:")) + 2;
    int msgEnd = response.indexOf("\r\n", msgStart);
    
    if (msgStart != -1 && msgEnd != -1) {
      String smsContent = response.substring(msgStart, msgEnd);
      processCommands(smsContent);
    }
    
    // Delete all read messages
    sendATCommand("AT+CMGD=1,1", 1000);
  }
}

/**
 * Processes SMS commands from owner
 *  - Parses message for recognized commands
 *  - Executes appropriate system action based on command
 *  - Sends confirmation response for each command
 *  @param message The SMS message content to process
 */
void processCommands(String message) {
  message.toLowerCase(); // Convert to lowercase for easier comparison
  
  Serial.println("Processing command: " + message);
  
  if (message.indexOf("status") != -1) {
    // Read the saved sleep mode state from preferences
    preferences.begin("security", true);
    bool savedSleepMode = preferences.getBool("sleepMode", false);
    preferences.end();
    
    // Comprehensive status report
    String statusMsg = "SYSTEM STATUS REPORT:\n";
    statusMsg += "Motorcycle: " + String(motorcycleEnabled ? "ENABLED" : "DISABLED") + "\n";
    statusMsg += "Alerts: " + String(alertsEnabled ? "ACTIVE" : "INACTIVE") + "\n";
    statusMsg += "Notifications: " + String(alertsEnabled ? "ON" : "OFF") + "\n";
    statusMsg += "Tracking: " + String(trackingEnabled ? "ENABLED" : "DISABLED") + "\n";
    statusMsg += "Sleep Mode: " + String(savedSleepMode ? "SLEEP" : "WAKE") + "\n";
    
    // Detailed system component status
    statusMsg += "\nSYSTEM COMPONENTS:\n";
    statusMsg += "MPU6050: " + String(mpu_ok ? "OPERATIONAL" : "FAULT") + "\n";
    statusMsg += "SIM800L: " + String(sim_ok ? "CONNECTED" : "DISCONNECTED") + "\n";
    statusMsg += "GPS: " + String(gps_ok ? "ACTIVE" : "NO FIX") + "\n";
    
    // Add GPS location if available
    if (gps.location.isValid()) {
      char locationStr[100];
      sprintf(locationStr, "\nLocation: %.6f, %.6f\n", 
              gps.location.lat(), gps.location.lng());
      statusMsg += locationStr;
      
      // Additional location details
      statusMsg += "Speed: " + String(gps.speed.kmph(), 1) + " km/h\n";
      statusMsg += "Satellites: " + String(gps.satellites.value());
    } else {
      statusMsg += "\nLocation: Not Available";
    }
    
    sendSMS(statusMsg);
  }
  
  else if (message.indexOf("track on") != -1) {
    // Enable tracking mode
    trackingEnabled = true;
    sendSMS("Tracking enabled - will send location every 10 seconds");
  }
  else if (message.indexOf("track off") != -1) {
    // Disable tracking mode
    trackingEnabled = false;
    sendSMS("Tracking disabled");
  }
  else if (message.indexOf("alerts on") != -1 || message.indexOf("notification on") != -1) {
    // Enable alerts
    alertsEnabled = true;
    sendSMS("Alert notifications enabled");
  }
  else if (message.indexOf("alerts off") != -1 || message.indexOf("notification off") != -1) {
    // Disable alerts
    alertsEnabled = false;
    movementDetected = false;
    tamperingDetected = false;
    sendSMS("Alert notifications disabled");
  }
  else if (message.indexOf("on") != -1) {
    // Turn on motorcycle
    controlMotorcycle(true);
  }
  else if (message.indexOf("off") != -1) {
    // Turn off motorcycle with safety check
    controlMotorcycle(false);
    // Note: Success message is sent in controlMotorcycle() if speed check passes
  }
  else if (message.indexOf("setnumber ") != -1) {
    // Extract new phone number
    int numberStart = message.indexOf("setnumber ") + 10;
    String newNumber = message.substring(numberStart);
    newNumber.trim(); // Remove any whitespace
    
    if (newNumber.length() > 0) {
      savePhoneNumber(newNumber);
      OWNER_PHONE = newNumber;
      sendSMS("Owner phone number updated to: " + newNumber);
    }
  }
  else if (message.indexOf("sleep") != -1) {
    // Enter sleep mode
    enableSleepMode();
  }
  else if (message.indexOf("wake") != -1) {
    // Wake from sleep mode
    wakeFromSleepMode();
  }
  else if (message.indexOf("acknowledge") != -1) {
    // Acknowledge alert
    waitingForAcknowledge = false;
    
    if (hornActivated) {
      activateHorn(false);
    }
    
    // Also hang up any ongoing call
    if (callInProgress) {
      sendATCommand("ATH", 1000);
      callInProgress = false;
    }
    
    sendSMS("Alert acknowledged. Horn deactivated.");
  }
  else if (message.indexOf("help") != -1) {
    // Send help message with available commands
    String helpMsg = "Available commands:\n";
    helpMsg += "status - Get system status\n";
    helpMsg += "on/off - Enable/disable motorcycle\n";
    helpMsg += "track on/off - Enable/disable GPS tracking\n";
    helpMsg += "alerts on/off - Enable/disable alert notifications\n";
    helpMsg += "setnumber <number> - Set new owner phone number\n";
    helpMsg += "sleep - Enter low power sleep mode\n";
    helpMsg += "wake - Wake from sleep mode\n";
    helpMsg += "acknowledge - Acknowledge alert and stop horn";
    
    sendSMS(helpMsg);
  }
}

/**
 * Controls motorcycle ignition relay with safety checks
 *  - Allows immediate enabling of motorcycle
 *  - Performs speed check before disabling for safety
 *  - Sets pending disable flag if speed is too high
 *  @param enable True to enable, False to disable with safety check
 */
void controlMotorcycle(bool enable) {
  // If enabling, always do it immediately
  if (enable) {
    motorcycleEnabled = true;
    digitalWrite(RELAY_KILL_PIN, HIGH);
    disablePending = false; // Clear any pending disable request
    sendSMS("Motorcycle enabled");
    return;
  }
  
  // If trying to disable the motorcycle, check speed first
  if (!enable) {
    // Check if GPS has a valid reading
    if (gps.location.isValid() && gps.speed.isValid()) {
      float currentSpeed = gps.speed.kmph();
      
      // Only disable if speed is below the threshold
      if (currentSpeed > SAFE_DISABLE_SPEED) {
        disablePending = true; // Set flag for pending disable
        sendSMS("SAFETY ALERT: Cannot disable motorcycle at " + 
                String(currentSpeed, 1) + " km/h. Motorcycle will automatically disable when speed drops below " + 
                String(SAFE_DISABLE_SPEED, 1) + " km/h.");
        return; // Exit without disabling
      }
    }
    // If GPS is not valid, we should warn but still allow disable
    else if (gps_ok) {
      sendSMS("WARNING: Disabling motorcycle without valid speed data. Use caution!");
    }
  }
  
  // Proceed with disabling
  motorcycleEnabled = false;
  digitalWrite(RELAY_KILL_PIN, LOW);
  disablePending = false; // Clear the pending flag since we've completed the disable
  sendSMS("Motorcycle disabled successfully");
}

/**
 * Controls horn relay with pattern capability
 *  - Activates or deactivates the horn pattern feature
 *  - Sets initial horn state when activated
 *  @param activate True to begin horn pattern, False to silence horn
 */
void activateHorn(bool activate) {
  hornActivated = activate;
  
  if (activate) {
    // Initialize horn pattern
    hornState = true;
    lastHornToggle = millis();
    digitalWrite(RELAY_HORN_PIN, LOW); // Start with horn ON (LOW activates the horn)
  } else {
    // Turn off horn completely
    hornState = false;
    digitalWrite(RELAY_HORN_PIN, HIGH); // Make sure horn is OFF
  }
}
