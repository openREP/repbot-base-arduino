#include <Arduino.h>
#include <Servo.h>
#include <vector>

int incomingByte;
std::vector<byte> buffer;

uint8_t digitalChannelMap[] = {2, 3, 4, 5, 6, 7};
uint8_t pwmChannelMap[] = {9, 10, 11, 13};
Servo pwmPorts[] = {Servo(), Servo(), Servo(), Servo()};

/**
 * Digital Ports: 2, 3, 4, 5, 6, 7
 * PWM Ports: 9, 10, 11, 13
 * Analog In ports: 0, 1, 2, 3, 4, 5
 */

/**
 * Serial Protocol
 * (Inspired by Pololu protocol)
 * 
 * Rules
 * -----
 * - Command bytes MUST have their most significant bits set (i.e. 128 - 255) while data bytes MUST have
 *   their MSB cleared 0 (i.e. 0 - 127)
 * - Query commands are used to obtain data. These commands all return data
 * - Set Parameter commands are used to set parameters
 * - Set Port commands are used to set port values
 * 
 * 0x81: Get Signature
 */

void processBuffer();

void setup() {
    // Set up the ports in their default state
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);

    // Set up the PWM pins as Servo objects
    pwmPorts[0].attach(9);
    pwmPorts[1].attach(10);
    pwmPorts[2].attach(11);
    pwmPorts[3].attach(13);
    
    Serial.begin(115200);
    while (!Serial) {
        // Wait for Serial port to be ready
    }
}

void loop() {
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        buffer.push_back((byte)incomingByte);
        processBuffer();
    }
}

/**
 * Ensure the buffer is valid (i.e. is either empty, or has the first byte as a command)
 */
void ensureValidBuffer() {
    if (buffer[0] < 0x80) {
        while (buffer.size() > 0 && buffer[0] < 0x80) {
            buffer.erase(buffer.begin());
        }
    }
}

/**
 * Command List
 */

/**
 * 0x81: Get Signature
 * Returns a 9 byte buffer that contains the current version info
 */
void processGetSignature() {
    // Strip the first element
    buffer.erase(buffer.begin());
    Serial.write("LabBot1.0");
}

/**
 * 0x82: Get Digital Input Values
 * Takes one data byte and returns one byte representing the state of the digital
 * input pins.
 * If bit n of the data byte is set, the corresponding value for channel n+2
 * (i.e. bit 0 = channel 2, bit 1 = channel 3, etc) will be put in bit n of the return
 * data byte. Other non-requested bits will be set to 0. 0x3F will read from ALL 
 * digital input pins
 */
void processGetDigitalInputValues() {
    buffer.erase(buffer.begin());
    if (buffer.size() >= 1) {
        byte data = buffer[0];
        byte output = 0;
        for (int i = 0; i < 6; i++) {
            if (data >> i & 0x1) {
                if (digitalRead(digitalChannelMap[i]) == HIGH) {
                    output |= (1 >> i);
                }
            }
        }
        buffer.erase(buffer.begin());

        Serial.write(output);
    }
}

/**
 * 0x83: Get Analog Input Values
 * Takes one data byte and returns 0 to 12 bytes representing the state of the analog
 * input pins.
 * If bit n of the data byte is set, the corresponding value for channel n will be
 * added to the return buffer. 0x3F will read from ALL analog pins.
 * For every analog pin requested, 2 bytes will be appended to the return buffer,
 * representing the analog value of the pin, from 0 - 1023, in big endian.
 */
void processGetAnalogInputValues() {
    buffer.erase(buffer.begin());
    if (buffer.size() >= 1) {
        byte data = buffer[0];
        for (int i = 0; i < 6; i++) {
            if (data >> i & 0x1) {
                int channelValue = analogRead(i);
                byte msb = (channelValue >> 8) & 0xFF;
                byte lsb = channelValue & 0xFF;
                Serial.write(msb);
                Serial.write(lsb);
            }
        }
        buffer.erase(buffer.begin());
    }
}

/**
 * 0xC0 - 0xC1: Set PWM 0
 * 0xC4 - 0xC5: Set PWM 1
 * 0xC8 - 0xC9: Set PWM 2
 * 0xCC - 0xCD: Set PWM 3
 * [ 1 1 0 0 ] [ X X Y Y ] [ Z Z Z Z Z Z Z Z ]
 * <-  0xC  ->  <ch> <dir> <-   data        ->
 * Set PWM values. Takes one data byte and does not return a value. This immediately sets
 * the PWM output of the specified channel (based on bits 3 and 4) in the appropriate 
 * direction (based on the 2 least significant bytes). A 00 in the direction bits represent
 * reverse, and a 01 represents forward
 */
void processSetPWMValues() {
    uint8_t channel = (buffer[0] >> 2) & 0x3;
    uint8_t direction = buffer[0] & 0x1; // 0 = reverse, 1 = forward
    buffer.erase(buffer.begin());
    if (buffer.size() >= 1) {
        uint8_t data = buffer[0];

        if (channel >= 0 && channel < 4 && data < 0x80) {
            // Valid channel
            int value = (int)data;
            // map(value, fromLow, fromHigh, toLow, toHigh)
            int angleDelta = (int)map(value, 0, 127, 0, 90);
            
            if (!direction) {
                angleDelta = -angleDelta;
            }
            int angle = 90 + angleDelta;
            
            // Set the servo
            pwmPorts[channel].write(angle);
        }

        buffer.erase(buffer.begin());
    }
}

void processBuffer() {
    ensureValidBuffer();
    if (buffer.size() == 0) {
        return;
    }

    // Figure out what to do with the command
    switch (buffer[0]) {
        case 0x81: 
            // Get Signature
            processGetSignature();
            break;
        case 0x82: 
            if (buffer.size() < 2) return;
            processGetDigitalInputValues();
            break;
        case 0x83:
            if (buffer.size() < 2) return;
            processGetAnalogInputValues();
            break;

        case 0xC0: // PWM0 - Reverse
        case 0xC1: // PWM0 - Forward
        case 0xC4: // PWM1 - Reverse
        case 0xC5: // PWM1 - Forward
        case 0xC8: // PWM2 - Reverse
        case 0xC9: // PWM2 - Forward
        case 0xCC: // PWM3 - Reverse
        case 0xCD: // PWM3 - Forward
            if (buffer.size() < 2) return;
            processSetPWMValues();
            break;
        default: 
            // Invalid command, strip this and then make the buffer valid again
            buffer.erase(buffer.begin());
            ensureValidBuffer();
    }
}

