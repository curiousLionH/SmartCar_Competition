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

float prev_center;
float prev_left;
float prev_right;

float ir_r_value;
float ir_l_value;

int state;

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

float cur_steering;
float cur_speed;
float compute_steering;
float compute_speed;

float max_pwm;
float min_pwm;

// 미션용 변수 =============================================================================
// 정지선 검출
int cnt_IR_BOTH;
unsigned long last_stop_line_time;

// R, L 방향 IR 센서가 연속으로 검출되는 경우
int cnt_IR_R;
int cnt_IR_L;
int cnt_IR_max = 50; // back을 하는 max 검출 카운트

<<<<<<< HEAD
int cnt_IR_max = 50; // back을 하는 max 검출 카운트

unsigned long last_stop_line_time = 0;

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

    float angle = cur_steering * angle_limit;
    int servoAngle = angle + 90;
    servoAngle += angle_offset;

    servoAngle = constrain(servoAngle, 0, 180);
    servo.write(servoAngle);
}

// 뒷바퀴 모터회전
void SetSpeed(float speed)
{
    speed = constrain(speed, -1, 0.6);

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

void line_tracing()
{ // 기본주행
    // 후진은 위험한 상황이니까 전진보다 먼저 고려
    if (cnt_IR_R > cnt_IR_max)
    {
        // 후진
        while (ir_r_value <= detect_ir)
        {
            SetSteering(0.6);
            SetSpeed(-0.5);
        }
        cnt_IR_R = 0;
    }
    else if (cnt_IR_L > cnt_IR_max)
    {
        // 후진
        while (ir_l_value <= detect_ir)
        {
            SetSteering(-0.6);
            SetSpeed(-0.5);
        }
        cnt_IR_L = 0;
    }

    else if (ir_r_value <= detect_ir)
    { // 오른쪽 차선이 검출된 경우
        compute_steering = -1;
        compute_speed = 0.3;
        cnt_IR_R++;
    }

    else if (ir_l_value <= detect_ir)
    { //왼쪽 차선이 검출된 경우
        compute_steering = 1;
        compute_speed = 0.3;
        cnt_IR_L++;
    }

    else if (ir_r_value >= detect_ir && ir_l_value >= detect_ir)
    { //차선이 검출되지 않을 경우 직진
        compute_steering = 0;
        compute_speed = 1;
        cnt_IR_R = 0;
        cnt_IR_L = 0;
    }
}

void _start()
{
    if (center > center_start)
    {
        line_tracing();
    }
}

void parking_p()
{
    // IR 센서에 감지 될 때 까지 전진
    while (ir_sensing(IR_R) > detect_ir && ir_sensing(IR_L) > detect_ir)
    {
        compute_steering = 0.5;
        compute_speed = 0.5;
        SetSteering(compute_steering);
        SetSpeed(compute_speed);
    }
}

    // 앞까지 거리가 25cm보다 작은 동안
    while (GetDistance(FC_TRIG, FC_ECHO) < 300)
    {
        right = GetDistance(R_TRIG, R_ECHO);
        if (right > 50)
        {
            // 오른쪽 뒤로
            compute_steering = 0.5;
            compute_speed = -0.5;
        }
        else if (right < 30)
        {
            // 왼쪽 뒤로
            compute_steering = 0.5;
            compute_speed = -0.5;
        }
        else
        {
            // 그냥 뒤로
            compute_steering = 0;
            compute_speed = -0.5;
        }
        SetSteering(compute_steering);
        SetSpeed(compute_speed);
    }
    SetSteering(0);
    SetSpeed(0);
    delay(5000);
}

void parking_t1()
{
    // 1. 좌회전
    if (millis() - last_stop_line_time > 100 && millis() - last_stop_line_time < 400)
    {
        compute_steering = -1;
        compute_speed = 0.1;
    }
    else
    {
        line_tracing();
    }
}

void parking_t2()
{
    if (right > side_detect)
    {
        compute_steering = compute_steering + 0.1;
        compute_speed = -0.5;
    }
    else
    {
        compute_steering = compute_steering - 0.1;
        compute_speed = -0.5;
    }
}

void parking_t3()
{
    // 3. 전진
    line_tracing();
}

void obstacle()
{
    if (center < center_stop && left < side_detect && right < side_detect)
    {
        compute_speed = 0;
        compute_steering = 0;
    }
    else if (center < center_detect && ir_l_value >= detect_ir)
    { // 장애물 발견 & 왼쪽 차선 안보임
        compute_steering = -1;
        compute_speed = 0.2;
        obstacle_cnt++;
    }
    else if (obstacle_cnt > 0 && obstacle_cnt < 100 && ir_l_value > detect_ir)
    {
        compute_steering = -1;
        compute_speed = 0.2;
        obstacle_cnt++;
    }
    else if (obstacle_cnt >= 100 && obstacle_cnt < 150)
    {
        compute_steering = 0.6;
        compute_speed = 0.2;
        obstacle_cnt++;
    }
    //    else if (ir_sensing(IR_R) > detect_ir && right > 50 && right < side_detect)
    //    { // 오른쪽에 장애물 있는 상태
    //        compute_steering = 0.3;
    //        cur_dir = 1;
    //        compute_speed = 0.1;
    //    }
    //    else if (obstacle_cnt > 0 && right > side_detect)
    //    { // 오른쪽에 장애물 없어진 상태
    //        compute_steering = 1;
    //        cur_dir = 1;
    //        compute_speed = 0.2;
    //    }
    else
    {
        line_tracing();
        obstacle_cnt++;
    }
}

bool CheckStopLine()
{
    // 방금 전에 정지선을 지나 온 경우
    if (millis() - last_stop_line_time < 5000)
    {
        return false;
    }

    if (ir_r_value <= detect_ir && ir_l_value <= detect_ir)
    {
        cnt_IR_BOTH++;
    }
    else
    {
        cnt_IR_BOTH = 0;
    }

    if (cnt_IR_BOTH >= 5)
    {
        last_stop_line_time = millis();
        return true;
    }
    return false;
}

void auto_driving(int state)
{
    switch (state)
    {
    case 0: // 출발
        _start();
        break;
    case 1: // 평행주차        
        parking_p1();
        break;
<<<<<<< HEAD
    case 2:
        parking_p2();
    case 3: // 8자 주행 2
        line_tracing();
        break;
    case 4: // T 주차 1
        parking_t1();
        break;
    case 5: // T 주차 2
        parking_t2();
        break;
    case 6: // T 주차 3
        parking_t3();
        break;
    case 7: // 버스 피하기
        obstacle();
        break;
    }
}
>>>>>>> 545a41ecf03996457df898b5f718c24ee3ccafca

void setup()
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
    state = 0;
}

void loop()
{
    prev_center = center;
    prev_left = left;
    prev_right = right;

    compute_steering = cur_steering;
    compute_speed = cur_speed;

    center = GetDistance(FC_TRIG, FC_ECHO);
    left = GetDistance(L_TRIG, L_ECHO);
    right = GetDistance(R_TRIG, R_ECHO);

    ir_r_value = ir_sensing(IR_R);
    ir_l_value = ir_sensing(IR_L);

    if (CheckStopLine())
    {
        state += 1;
    }

    auto_driving(state);

    SetSpeed(compute_speed);
    SetSteering(compute_steering);
}