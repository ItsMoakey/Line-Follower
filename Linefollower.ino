// Motor control pins
#define ENA 3
#define IN1 5
#define IN2 6
#define IN3 9
#define IN4 10
#define ENB 11

// Sensor pins
#define NUM_SENSORS 8
int sensorPins[NUM_SENSORS] = {2, A0, A1, A2, A3, A4, A5, 12};

// PID parameters
float Kp = 30.0;  // Proportional gain
float Ki = 0.0;   // Integral gain (start with 0)
float Kd = 10.0;  // Derivative gain

int baseSpeed = 120;  // Base speed (0-255)
float lastError = 0;
float integral = 0;

void setup() {
    Serial.begin(9600);
    
    // Set motor control pins as outputs
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Set sensor pins as inputs
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    int sensorValues[NUM_SENSORS];
    int position = 0;
    int sum = 0;
    int activeSensors = 0;

    // Read sensor values
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
        if (sensorValues[i] == 0) { // Line detected (black)
            position += i * 100;  // Assign weights (0, 100, 200... 700)
            sum += 100;
            activeSensors++;
        }
    }

    // Calculate error
    float error;
    if (activeSensors > 0) {
        error = (position / (float)sum) - 350.0;  // Center at 350 (midpoint of 0-700 scale)
    } else {
        error = lastError; // No line detected, use last error
    }

    // PID calculations
    integral += error;
    float derivative = error - lastError;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    lastError = error;

    // Adjust motor speeds
    int leftSpeed = constrain(baseSpeed + correction, 0, 255);
    int rightSpeed = constrain(baseSpeed - correction, 0, 255);

    moveMotors(leftSpeed, rightSpeed);
}

void moveMotors(int leftSpeed, int rightSpeed) {
    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }

    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }

    analogWrite(ENA, abs(leftSpeed));
    analogWrite(ENB, abs(rightSpeed));
}
