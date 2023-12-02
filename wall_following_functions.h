#define RMotorA 3   // Right motor forward direction
#define RMotorB 4   // Right motor backward direction
#define RMotorPWM 6 // Right motor PWM

#define LMotorA 2   // Left motor forward direction
#define LMotorB 7   // Left motor backward direction
#define LMotorPWM 5 // Left motor PWM

#define MAX_SPEED 80

uint8_t trigPin_fr = 8; // TRIG pin front right
uint8_t echoPin_fr = 9; // ECHO pin front right

uint8_t trigPin_br = 10; // TRIG pin back right
uint8_t echoPin_br = 11; // ECHO pin back right

int turnSpeed = 50;
int MotorBasespeed = 50;
int speed_offset = 7;

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;


int desiredDistance = 4; // Desired distance from the wall (ex: 4 cm)
int D1; // not the exact distance from the wall (when distance=0 or 50)

double wkp = 1.0;
double wkd = 0.1;
double wki = 0.0001;

double errorSum = 0;
double lastError = 0;

// Function to initialize motors
void initializeMotors()
{
    // Initialize Left Motor
    pinMode(LMotorA, OUTPUT);
    pinMode(LMotorB, OUTPUT);
    pinMode(LMotorPWM, OUTPUT);

    // Initialize Right Motor
    pinMode(RMotorA, OUTPUT);
    pinMode(RMotorB, OUTPUT);
    pinMode(RMotorPWM, OUTPUT);
}

// Function to initialize ultrasonic sensors
void initializeUltrasonicSensors()
{
    pinMode(trigPin_fr, OUTPUT);
    pinMode(echoPin_fr, INPUT);

    pinMode(trigPin_br, OUTPUT);
    pinMode(echoPin_br, INPUT);
}

// Function to get distance from ultrasonic sensors
int get_distance(uint8_t trigPin, uint8_t echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    int distance = (duration * 0.034) / 2;

    return distance;
}

// Enables motors to move forward
void set_forward()
{
    digitalWrite(LMotorA, HIGH);
    digitalWrite(RMotorA, HIGH);
    digitalWrite(LMotorB, LOW);
    digitalWrite(RMotorB, LOW);
}

// Function to give shock to motors (forward direction)
void shock_forward()
{
    set_forward();
    analogWrite(LMotorPWM, 255);
    analogWrite(RMotorPWM, 255);
    delay(10);
    analogWrite(LMotorPWM, 0);
    analogWrite(RMotorPWM, 0);
}

// Function to adjust speed of motors according to the PID signals
void set_speed()
{
    analogWrite(LMotorPWM, LMotorSpeed);
    analogWrite(RMotorPWM, RMotorSpeed);
}

// Function to calculate the speed according to the Error
void wPID_control()
{
    // Calculate error
    double error = D1 - desiredDistance;

    // Update error sum for integral term
    errorSum += error;

    // Calculate derivative term
    double errorDiff = error - lastError;

    // Calculate PID output
    double output = wkp * error + wki * errorSum + wkd * errorDiff;

    // Map output to motor speed (0 to 255)
    int speedAdjust = map(output, -255, 255, 0, 255);

    // Apply motor speeds
    // analogWrite(LMotorPWM, motorSpeed);
    // analogWrite(RMotorPWM, motorSpeed);

    // Set the current error for next iteration
    lastError = error;

    // Print the motor speed
    Serial.print(" speedAdjust ");
    Serial.print(speedAdjust);
    Serial.println(" ");

    // Set motor speeds
    LMotorSpeed = MotorBasespeed - speedAdjust + speed_offset;
    RMotorSpeed = MotorBasespeed + speedAdjust - speed_offset;


    // Keep the speeds in the limits
    if (LMotorSpeed < 0)
    {
        LMotorSpeed = 0;
    }
    if (RMotorSpeed < 0)
    {
        RMotorSpeed = 0;
    }
    if (LMotorSpeed > MAX_SPEED)
    {
        LMotorSpeed = MAX_SPEED;
    }
    if (RMotorSpeed > MAX_SPEED)
    {
        RMotorSpeed = MAX_SPEED;
    }

    Serial.print("LMS: ");
    Serial.print(LMotorSpeed);
    Serial.print(" cm/s ,  RMS: ");
    Serial.print(RMotorSpeed);
    Serial.println(" cm/s");
}


// Function to move forward the robot according to PID signals
void wpid_foward()
{
    // Read distance from ultrasonic sensor
    int right_distance = get_distance(trigPin_br, echoPin_br);
    
    D1 = right_distance;

    // Keep the limits for distance from the right wall
    if (D1 > 50 || D1 == 0)
    {
        D1 = 50;
    }
    
    // Adjust speed according to the PID signals
    wPID_control();

    // Enable pins for forward direction and set speed
    set_forward();
    set_speed();
    
    delay(100);
}


