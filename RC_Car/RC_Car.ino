#include <Servo.h>
Servo servo;

const int SERVO1_PIN = 9; // 서보모터1 연결핀
const int IR_R = A3;      //  적외선센서 우측 핀
const int IR_L = A4;      // 적외선센서 좌측 핀

const int M1_PWM = 5;  // DC모터1 PWM 핀 왼
const int M1_DIR1 = 7; // DC모터1 DIR1 핀
const int M1_DIR2 = 8; // DC모터 1 DIR2 핀

const int M2_PWM = 6;   // DC모터2 PWM 핀
const int M2_DIR1 = 11; // DC모터2 DIR1 핀
const int M2_DIR2 = 12; // DC모터2 DIR2 핀

const int FC_TRIG = 13; // 전방 초음파 센서 TRIG 핀
const int FC_ECHO = 10; // 전방 초음파 센서 ECHO 핀
const int L_TRIG = A2;  // 좌측 초음파 센서 TRIG 핀
const int L_ECHO = A1;  // 좌측 초음파 센서 ECHO 핀
const int R_TRIG = 2;   // 우측 초음파 센서 TRIG 핀
const int R_ECHO = A5;  // 우측 초음파 센서 ECHO 핀

const int MAX_DISTANCE = 2000; // 초음파 센서의 최대 감지거리

float center;
float left;
float right;

int state = 3;

// 자동차 튜닝 파라미터 =====================================================================
int detect_ir = 30; // 검출선이 흰색과 검정색 비교

int punch_pwm = 220; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 50; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)

int max_ai_pwm = 130; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 70;  // 자율주행 모터 최소 출력 (0 ~ 255)

int angle_offset = -5; // 서보 모터 중앙각 오프셋 (단위: 도)
int angle_limit = 55;  // 서보 모터 회전 제한 각 (단위: 도)

int center_detect = 200; // 전방 감지 거리 (단위: mm)
int center_start = 160;  // 전방 출발 거리 (단위: mm)
int center_stop = 70;    // 전방 멈춤 거리 (단위: mm)

int side_detect = 100; // 좌우 감지 거리 (단위: mm)

int prev_dir = 0;
float prev_speed;
float prev_steering = -1000;
int cur_dir = 0;
float compute_steering;
float compute_speed;

float steer_gain = 0.5;

float max_pwm;
float min_pwm;

// 미션용 변수 =============================================================================
bool start_done = false;
bool parking_p_end = false;
float diff_RL = 0;
bool flag1 = true;
bool flag2 = true;
bool flag3 = true;

// 정지선 검출
bool stopLine = false;
int cnt_IR_R;
int cnt_IR_L;

bool obstacle_end = false;

// 초음파 거리측정
float GetDistance(int trig, int echo)
{
    digitalWrite(trig, LOW);
    delayMicroseconds(4);
    digitalWrite(trig, HIGH);
    delayMicroseconds(20);
    digitalWrite(trig, LOW);

    unsigned long duration = pulseIn(echo, HIGH, 5000);
    if (duration == 0)
        return MAX_DISTANCE;
    else
        return duration * 0.17; // 음속 340m/s
}

int ir_sensing(int pin)
{
    return analogRead(pin);
}

// 앞바퀴 조향
void SetSteering(float steering)
{
    cur_steering = constrain(steering, -1, 1); // constrain -1~ 1 값으로 제한
//    Serial.println(cur_steering);
    float angle = cur_steering * angle_limit;
    int servoAngle = angle + 90;
    servoAngle += angle_offset;

    servoAngle = constrain(servoAngle, 0, 180);
    servo.write(servoAngle);
}

// 뒷바퀴 모터회전
void SetSpeed(float speed)
{
    speed = constrain(speed, -1, 1);

    if ((cur_speed * speed < 0)            // 움직이는 중 반대 방향 명령이거나
        || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
    {
        cur_speed = 0;
        digitalWrite(M1_PWM, HIGH);
        digitalWrite(M1_DIR1, LOW);
        digitalWrite(M1_DIR2, LOW);

        digitalWrite(M2_PWM, HIGH);
        digitalWrite(M2_DIR1, LOW);
        digitalWrite(M2_DIR2, LOW);

        if (stop_time > 0)
            delay(stop_time);
    }

    if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
    {
        if (punch_time > 0)
        {
            if (speed > 0)
            {
                analogWrite(M1_PWM, punch_pwm);
                digitalWrite(M1_DIR1, HIGH);
                digitalWrite(M1_DIR2, LOW);

                analogWrite(M2_PWM, punch_pwm);
                digitalWrite(M2_DIR1, HIGH);
                digitalWrite(M2_DIR2, LOW);
            }
            else if (speed < 0)
            {
                analogWrite(M1_PWM, punch_pwm);
                digitalWrite(M1_DIR1, LOW);
                digitalWrite(M1_DIR2, HIGH);

                analogWrite(M2_PWM, punch_pwm);
                digitalWrite(M2_DIR1, LOW);
                digitalWrite(M2_DIR2, HIGH);
            }
            delay(punch_time);
        }
    }

    if (speed != 0) // 명령이 정지가 아니라면
    {
        int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm; // 0 ~ 255로 변환

        if (speed > 0)
        {
            analogWrite(M1_PWM, pwm);
            digitalWrite(M1_DIR1, HIGH);
            digitalWrite(M1_DIR2, LOW);

            analogWrite(M2_PWM, pwm);
            digitalWrite(M2_DIR1, HIGH);
            digitalWrite(M2_DIR2, LOW);
        }
        else if (speed < 0)
        {
            analogWrite(M1_PWM, pwm);
            digitalWrite(M1_DIR1, LOW);
            digitalWrite(M1_DIR2, HIGH);

            analogWrite(M2_PWM, pwm);
            digitalWrite(M2_DIR1, LOW);
            digitalWrite(M2_DIR2, HIGH);
        }
    }
    cur_speed = speed;
}
void straight()
{ // 기본주행
//    Serial.print("Right : ");
//    Serial.print(ir_sensing(IR_R));
//    Serial.print("    Left : ");
//    Serial.println(ir_sensing(IR_L));

    if (center < center_detect && ir_sensing(IR_L) >= detect_ir)
    { // 장애물 발견
        compute_steering = -1;
        cur_dir = -1;
        compute_speed = 0.3;
    }

    else if (ir_sensing(IR_R) <= detect_ir)
    { // 오른쪽 차선이 검출된 경우
        Serial.println("Left");
        compute_steering = -0.6;
        cur_dir = -1;
        compute_speed = 0.1;
    }

    else if (ir_sensing(IR_L) <= detect_ir)
    { //왼쪽 차선이 검출된 경우
        Serial.println("Right");
        compute_steering = 0.6;
        cur_dir = 1;
        compute_speed = 0.1;
    }

    else if(ir_sensing(IR_R) >= detect_ir && ir_sensing(IR_L) >= detect_ir)
        { //차선이 검출되지 않을 경우 직진
            Serial.println("Straight");
            compute_steering = 0;
            cur_dir = 0;
            compute_speed = 1;
        }

    if (cur_dir == prev_dir && cur_dir == 1)
        {
        compute_steering = prev_steering + 0.1;
        }
    else if (cur_dir == prev_dir && cur_dir == -1)
        {
        compute_steering = prev_steering - 0.1;
        }
    else{   // 다른 dir
        compute_steering = 0;    
        } 
}
void start()
{
    if (center < center_detect){   // 앞에 막혀 있을 때
      Serial.print("Wait!   ");
      Serial.println(center);
    }
    else{
        Serial.println("Start!");
        straight();
    }
}

void parking_p()
{
    compute_speed = 0.1;
    compute_steering = 0;

    //    while(!parking_p_end){
    while (false)
    {
        center = GetDistance(FC_TRIG, FC_ECHO);
        left = GetDistance(L_TRIG, L_ECHO);
        right = GetDistance(R_TRIG, R_ECHO);
        if (left > 100)
        {
            straight();
        }
        else
        {
            if (flag1 && right < 50 && center > 50)
            {
                compute_steering += 0.3;
            }
            else if (flag2 && center < 50 && center > 20)
            {
                flag1 = false;
                compute_steering -= 0.2;
            }
            else if (flag3 && center < 20)
            {
                flag2 = false;
                compute_speed = -0.1;
                compute_steering = 0;
            }
            else
            {
                flag3 = false;
                parking_p_end = true;
            }
        }
        SetSpeed(compute_speed);
        SetSteering(compute_steering);
    }

    // 주차 끝나고 나오기
    while (true)
    {
        center = GetDistance(FC_TRIG, FC_ECHO);
        left = GetDistance(L_TRIG, L_ECHO);
        right = GetDistance(R_TRIG, R_ECHO);
        if (left < 40)
        {
            break;
        }
        diff_RL = right - left;
        compute_steering = constrain(diff_RL / 800, -1, 1);

        if (center < 100)
        {
            compute_steering = constrain(compute_steering * 1.3, -1, 1);
        }

        else if (ir_sensing(IR_L) <= detect_ir)
        { //왼쪽 차선이 검출된 경우
            //            Serial.println("Right");
            compute_steering = 1;
            //            compute_speed = 0.1;
        }

        SetSpeed(compute_speed);
        SetSteering(compute_steering);
    }

    while (true)
    {
        SetSpeed(0.4);
        SetSteering(0);
    }
}

void _end()
{
    if (center < 20){   // 앞에 막혀 있을 때
      compute_steering = 0;
      compute_speed = 0;
    }
    else{
        straight();
    }
}

void driving() {
    compute_steering = cur_steering;
    compute_speed = cur_speed;
    prev_steering = cur_steering;
    prev_dir = cur_dir;

    center = GetDistance(FC_TRIG, FC_ECHO);
    left = GetDistance(L_TRIG, L_ECHO);
    right = GetDistance(R_TRIG, R_ECHO);
    straight();
    //    if (state == 0) {
    //        straight();
    //        if (ir_sensing(IR_R) <= detect_ir && ir_sensing(IR_L) <= detect_ir) { //양쪽 차선이 검출된 경우
    //         state = 2;
    //        }
    //        if (left <= side_detect && right <= side_detect) {
    //         state = 1;
    //        }
    //        if (center <= center_detect) {
    //         state = 3;
    //        }
    //    }
    if (cur_dir == prev_dir && cur_dir == 1)
    {
        compute_steering = prev_steering + 0.1;
    }
    else if (cur_dir == prev_dir && cur_dir == -1)
    {
        compute_steering = prev_steering - 0.1;
    }
//    if (prev_steering != -1000){
//        compute_steering = compute_steering*steer_gain + prev_steering*(1-steer_gain);
//    }
}

void auto_driving(int state){
    switch(state){
        case 0:     // 출발
            start();
            break;
        case 1:     // 평행주차
            parking_p();
            break;
        case 2:     // T 주차
//            parking_t();
              break;
        case 3:     // 장애물 회피
            straight();
            break;
        case 4:     // 종료
            _end();
            break;
    }
}
bool CheckStopLine()
{
    if (ir_sensing(IR_R) <= detect_ir && ir_sensing(IR_L) <= detect_ir)
    {
        cnt_IR_R++;
        cnt_IR_L++;
    }
    else
    {
        cnt_IR_R = 0;
        cnt_IR_L = 0;
    }

    if (cnt_IR_R == cnt_IR_L && cnt_IR_R >= 5)
    {
        return true;
    }
    return false;
}

void setup()
{

    Serial.begin(115200);
    servo.attach(SERVO1_PIN); //서보모터 초기화

    pinMode(IR_R, INPUT);
    pinMode(IR_L, INPUT);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR1, OUTPUT);
    pinMode(M1_DIR2, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR1, OUTPUT);
    pinMode(M2_DIR2, OUTPUT);

    pinMode(FC_TRIG, OUTPUT);
    pinMode(FC_ECHO, INPUT);

    pinMode(L_TRIG, OUTPUT);
    pinMode(L_ECHO, INPUT);

    pinMode(R_TRIG, OUTPUT);
    pinMode(R_ECHO, INPUT);

    max_pwm = max_ai_pwm;
    min_pwm = min_ai_pwm;

    SetSteering(0);
    SetSpeed(0);
}

void loop() {
    if (CheckStopLine()){
        delay(3000);
        state += 1;
    }
    Serial.println(state);

    prev_speed = compute_speed;
    prev_steering = compute_steering;
    prev_dir = cur_dir;

    center = GetDistance(FC_TRIG, FC_ECHO);
    left = GetDistance(L_TRIG, L_ECHO);
    right = GetDistance(R_TRIG, R_ECHO);

    auto_driving(state);
    // Serial.println("done1!");
    SetSpeed(compute_speed);
    SetSteering(compute_steering);
//    Serial.println("done!");
}
