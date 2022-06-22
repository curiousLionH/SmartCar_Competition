const int IR_R = A3;  //  적외선센서 우측 핀
const int IR_L = A4;  // 적외선센서 좌측 핀

void setup() {
    Serial.begin(115200);
    pinMode(IR_R, INPUT);
    pinMode(IR_L, INPUT);
}

void loop() {

    Serial.print("Left : "); Serial.print(analogRead(IR_L));
    Serial.print("\tRight : "); Serial.println(analogRead(IR_R));
    delay(200);
    
}
