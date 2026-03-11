void setup() {
    Serial.begin(115200);

    // motor 1
    pinMode(2, OUTPUT); // step
    pinMode(3, OUTPUT); // dir

    // motor 2
    pinMode(4, OUTPUT); // step
    pinMode(5, OUTPUT); // dir

    // motor 3
    pinMode(6, OUTPUT); // step
    pinMode(7, OUTPUT); // dir

}

void step(uint8_t step_pin) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(step_pin, LOW);
}

void loop() {
    if (Serial.available() > 0) {
        uint8_t b = Serial.read();
        if (b != 0xAA) return;

        uint8_t buf[64];
        int len = 0;
        unsigned long start = millis();

        while (millis() - start < 100) {
            if (Serial.available() > 0) {
                uint8_t c = Serial.read();
                if (c == 0x55) break;
                buf[len++] = c;
                if (len >= 64) break;
            }
        }

        if (len < 3) return;

        uint8_t motor1 = buf[0];
        uint8_t motor2 = buf[1];
        uint8_t motor3 = buf[2];

        if (motor1 == 0x01) {
            digitalWrite(3, HIGH);
            step(2);
        } else if (motor1 == 0x02) {
            digitalWrite(3, LOW);
            step(2);
        }

        if (motor2 == 0x01) {
            digitalWrite(5, HIGH);
            step(4);
        } else if (motor2 == 0x02) {
            digitalWrite(5, LOW);
            step(4);
        }

        if (motor3 == 0x01) {
            digitalWrite(7, HIGH);
            step(6);
        } else if (motor3 == 0x02) {
            digitalWrite(7, LOW);
            step(6);
        }

        // Serial.write(buf, len);
    }
}