#include <Servo.h>

// --- Configuration ---
// Assign 6 PWM-capable pins on the Arduino Mega.
// Pins 2 through 13 are all good choices.
const int thrusterPins[6] = {2, 3, 4, 5, 6, 7};

// Create an array of Servo objects
Servo thrusters[6];

void setup() {
    // Start serial communication
    Serial.begin(115200);
    Serial.println("Arduino Mega ESC Controller Ready. Waiting for commands...");

    // Attach each thruster's pin to a servo object
    for (int i = 0; i < 6; i++) {
        thrusters[i].attach(thrusterPins[i]);
    }

    // Arm the ESCs by sending a neutral signal (1500µs)
    Serial.println("Arming ESCs...");
    for (int i = 0; i < 6; i++) {
        thrusters[i].writeMicroseconds(1500);
    }
    delay(2000); // Wait 2 seconds for ESCs to initialize
    Serial.println("ESCs armed.");
}

void loop() {
    // Check if data is available from the Raspberry Pi
    if (Serial.available() > 0) {
        // Read the incoming string up to the newline character
        String commandString = Serial.readStringUntil('\n');

        // Variables for parsing the string
        int lastIndex = 0;
        int thrusterIndex = 0;

        // Loop through the command string to find commas
        for (int i = 0; i < commandString.length(); i++) {
            if (commandString.charAt(i) == ',') {
                if (thrusterIndex < 6) {
                    // Extract the value between commas
                    String pwmValue = commandString.substring(lastIndex, i);
                    // Convert to an integer and send to the ESC
                    thrusters[thrusterIndex].writeMicroseconds(pwmValue.toInt());
                    
                    thrusterIndex++;
                    lastIndex = i + 1;
                }
            }
        }

        // Process the last value in the string (after the final comma)
        if (thrusterIndex < 6) {
            String pwmValue = commandString.substring(lastIndex);
            thrusters[thrusterIndex].writeMicroseconds(pwmValue.toInt());
        }
    }
}