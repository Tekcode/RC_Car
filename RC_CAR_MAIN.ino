#include <AFMotor.h> // Motor Drivers
#include <Ultrasonic.h> // Ultrasonic drivers

// Set FSM States
#define GOFORWARD 0
#define TURNLEFT 1
#define TURNRIGHT 2
#define GOFORWARD_WALL 3

#define STOPDISTANCE 70 // Distance in CM to stop the RC car. 30 CM = ~ 1 foot

// Arduino pins
#define echoPin A0
#define trigPin A2

// Forward declarations
void motor_control(unsigned char, unsigned long);
unsigned char detect_wall(void); // Detect an obstacle
Ultrasonic detect_range(trigPin, echoPin); // Initialize the ultrasonic class
AF_DCMotor frontwheels(1, MOTOR12_64KHZ); // Initialize front wheels
AF_DCMotor rearwheels(2, MOTOR12_64KHZ); // Initialize rear wheels

// Enable Debugging
const unsigned char DEBUG_ENABLE = 0;

typedef struct sensor_fsm {
	unsigned char out; // Set the output of the motors
	unsigned long delay; // Set the delay in between runs (in ms)
	unsigned char nextstate[2]; // The next state depending on the input
} sensor_fsm;

const sensor_fsm carstates[4] = {
	{GOFORWARD, 25, {GOFORWARD, TURNLEFT}},		// GOFORWARD
	{TURNLEFT, 2000, {GOFORWARD_WALL, TURNRIGHT}},		// TURN LEFT
	{TURNRIGHT, 2000, {GOFORWARD_WALL, TURNRIGHT}}, // TURN RIGHT
	{GOFORWARD, 25, {GOFORWARD, TURNRIGHT}}		// GOFOWARD if a wall is found
};

unsigned char sensor_state; // Input of sensor
unsigned char sensor_input; //Input of the sensor

void setup()
{
	// Start Debugging
	if (DEBUG_ENABLE == 1) {
		Serial.begin(115200); // Start Serial
		Serial.println("========== Debugging Beging ==========");
	}
	// End Debugging
	
	frontwheels.setSpeed(255); // Set front motor speed to max
	rearwheels.setSpeed(255); // Set rear motor speed to max

	sensor_state = 0; // Set state to GOFORWARD
	sensor_input = NULL; // No value until the sensor is read
}

void loop()
{
	motor_control(carstates[sensor_state].out, carstates[sensor_state].delay); // Output the motor to the current state, and delay in MS
	sensor_input = detect_wall(); // Set the next state
	sensor_state = carstates[sensor_state].nextstate[sensor_input]; // Set the next input
}

void motor_control(unsigned char fsm_output, unsigned long msDelay) {
	switch (fsm_output) {
		case 0:
			// Start Debugging
			if (DEBUG_ENABLE == 1) {
				Serial.println("Rear wheels go forward");
			}
			// End Debugging
			
			rearwheels.run(FORWARD);
			delay(msDelay); // Delay per the current state
			break;
		case 1:
			// Start Debugging
			if (DEBUG_ENABLE == 1) {
				Serial.println("Front wheels go left");
			}
			// End Debugging
				
			frontwheels.run(FORWARD);
			rearwheels.run(BACKWARD);
			delay(msDelay); // Delay per the current state
			frontwheels.run(RELEASE);
			break;
		case 2:
			// Start Debugging
			if (DEBUG_ENABLE == 1) {
				Serial.println("Front wheels go right");
			}
			// End Debugging
		
			frontwheels.run(BACKWARD);
			rearwheels.run(BACKWARD);
			delay(msDelay); // Delay per the current state
			frontwheels.run(RELEASE);
			break;
		case 4:
			// Start Debugging
			if (DEBUG_ENABLE == 1) {
				Serial.println("Rear wheels go forward");
			} 
			// End Debugging
		
			rearwheels.run(FORWARD);
			delay(msDelay); // Delay per the current state
			break;			
	}
}

unsigned char detect_wall() {
	long distance;
	distance = detect_range.Ranging(CM);
	// Start Debugging
	if (DEBUG_ENABLE == 1) {
		Serial.print(distance);
		Serial.println(" distance in CM");
	}
	// End Debugging
	
	// If the ultrasonic detector detects a range less than 20 CM, then change state to avoid obstacle
	if (distance > STOPDISTANCE) {
		// Start Debugging
		if (DEBUG_ENABLE == 1) {
			Serial.println("No obstacle detected");
		}
		// End Debugging	
				
		return 0; // Continue moving in the current direction
	} else if (distance <= STOPDISTANCE) {
		// Start Debugging
		if (DEBUG_ENABLE == 1) {
			Serial.println("Obstacle detected");
		}
		// End Debugging
				
		return 1; // Set the next state
	}
}