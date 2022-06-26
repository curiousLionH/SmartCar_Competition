#include <Servo.h>
#include "Pitches.h"
Servo servo;

#define SPEAKER_PIN 3

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
int detect_ir = 28; // 검출선이 흰색과 검정색 비교

int punch_pwm = 220; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 40; // 정지 마찰력 극복 시간 (단위 msec)
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
unsigned long last_stop_line_time = 0;

// R, L 방향 IR 센서가 연속으로 검출되는 경우
int cnt_IR_R;
int cnt_IR_L;

// 노래
unsigned long melody_t = 0;
int melody_index = 0;

int melody_road_201[] = {
    NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_G4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_G5, 0, 0,
    NOTE_A5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6, NOTE_G5, 0, 0,
    NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_A5, 0, NOTE_G5, 0,
    NOTE_C5, 0, NOTE_C5, 0, NOTE_C5, 0, NOTE_C5, 0,
    NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_G4, NOTE_D5,
    NOTE_C5, 0, NOTE_C5, 0, NOTE_C5, 0, NOTE_C5, 0,
    NOTE_G4, NOTE_D5, NOTE_E5, 0, NOTE_D5, 0, NOTE_C5, 0,
    NOTE_G5, 0, 0, NOTE_C6, 0, 0, NOTE_B5, 0,
    NOTE_A5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_C5, 0, NOTE_A5,
    NOTE_G5, 0, 0, NOTE_C6, 0, 0, NOTE_B5, 0,
    NOTE_A5, NOTE_G5, NOTE_E5, NOTE_C5, NOTE_G4, NOTE_D5, NOTE_C5, 0};

int melody_we_are_all_friends[] = { // 길이 264
    NOTE_E5, NOTE_G5, NOTE_G5, NOTE_G5, 0, NOTE_E5, NOTE_G5, 0,
    NOTE_E5, NOTE_G5, NOTE_G5, NOTE_G5, 0, NOTE_A5, NOTE_G5, 0,
    NOTE_C6, NOTE_C6, NOTE_C6, 0, NOTE_C6, NOTE_C6, NOTE_C6, 0,
    NOTE_C6, NOTE_A5, NOTE_A5, NOTE_A5, 0, NOTE_G5, NOTE_A5, 0,
    NOTE_F5, NOTE_F5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_A5, NOTE_B5, NOTE_B5,
    NOTE_C6, NOTE_GS5, 0, NOTE_GS5, 0, 0, 0, 0,
    NOTE_E6, NOTE_E6, NOTE_E6, NOTE_E6, 0, NOTE_C6, NOTE_E6, 0,
    NOTE_D6, 0, 0, 0, 0, 0, 0, 0,
    NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, 0, NOTE_E5, NOTE_G5, 0,
    NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, 0, NOTE_A5, NOTE_G5, 0,
    NOTE_C6, NOTE_C6, NOTE_C6, NOTE_C6, NOTE_C6, NOTE_C6, NOTE_C6, NOTE_C6,
    NOTE_C6, NOTE_A5, NOTE_A5, NOTE_A5, 0, NOTE_G5, NOTE_A5, 0,
    NOTE_C6, NOTE_C6, NOTE_D6, NOTE_D6, NOTE_E6, NOTE_E6, NOTE_F6, NOTE_G6,
    0, NOTE_G6, NOTE_G6, NOTE_G6, NOTE_G6, NOTE_F6, 0, 0,
    NOTE_E6, NOTE_E6, NOTE_E6, NOTE_E6, 0, NOTE_D6, NOTE_C6, 0,
    NOTE_D6, 0, 0, 0, 0, 0, 0, 0,
    NOTE_A5, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, 0, NOTE_C6, NOTE_B5,
    0, NOTE_C6, NOTE_D6, NOTE_D6, NOTE_D6, NOTE_B5, 0, 0,
    NOTE_B5, NOTE_A5, NOTE_B5, NOTE_A5, NOTE_B5, NOTE_C6, NOTE_D6, NOTE_C6,
    0, 0, NOTE_B5, 0, NOTE_A5, 0, 0, 0,
    NOTE_A5, NOTE_B5, NOTE_C6, NOTE_C6, 0, 0, 0, 0,
    NOTE_B5, NOTE_C6, NOTE_D6, NOTE_D6, 0, 0, 0, 0,
    NOTE_B5, NOTE_C6, NOTE_D6, NOTE_C6, NOTE_D6, NOTE_E6, 0, 0,
    NOTE_E6, 0, NOTE_F6, 0, NOTE_E6, 0, NOTE_D6, NOTE_D6,
    0, NOTE_C6, 0, 0, 0, 0, 0, 0,
    NOTE_A5, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, 0,
    NOTE_G5, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, 0,
    NOTE_E6, NOTE_E6, NOTE_E6, NOTE_F6, NOTE_G6, NOTE_A6, NOTE_G6, NOTE_F6,
    NOTE_E6, 0, NOTE_C6, 0, NOTE_D6, 0, 0, 0,
    NOTE_A5, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, 0,
    NOTE_G5, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, NOTE_B5, NOTE_C6, 0,
    NOTE_E6, NOTE_E6, NOTE_F6, NOTE_G6, NOTE_D6, NOTE_C6, NOTE_B5, 0,
    NOTE_C6, 0, 0, 0, 0, 0, 0, 0};

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

    float angle = cur_steering * angle_limit;
    int servoAngle = angle + 90;
    servoAngle += angle_offset;

    servoAngle = constrain(servoAngle, 0, 180);
    servo.write(servoAngle);
}

// // 뒷바퀴 모터회전 ***original 코드
void SetSpeed_original(float speed)
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

bool is_stop_time = false;
unsigned long stop_time_time = 0;

bool is_punch_time = false;
unsigned long punch_time_time = 0;

void SetSpeed(float speed, bool back = false)
{

   if (is_stop_time)
   {
       if (millis() - stop_time_time < stop_time)
       {

           cur_speed = 0;
           digitalWrite(M1_PWM, HIGH);
           digitalWrite(M1_DIR1, LOW);
           digitalWrite(M1_DIR2, LOW);

           digitalWrite(M2_PWM, HIGH);
           digitalWrite(M2_DIR1, LOW);
           digitalWrite(M2_DIR2, LOW);
       }
       else
       {
           is_stop_time = false;
       }
   }
   else if (is_punch_time)
   {
       if (millis() - punch_time_time < punch_time)
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
       }
       else
       {
           is_punch_time = false;
       }
   }
   else
   {
       speed = constrain(speed, -1, 1);
       if ((cur_speed * speed < 0)            // 움직이는 중 반대 방향 명령이거나
           || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
       {
           if (!is_stop_time)
           {
               stop_time_time = millis();
               is_stop_time = true;
               
               cur_speed = 0;
               digitalWrite(M1_PWM, HIGH);
               digitalWrite(M1_DIR1, LOW);
               digitalWrite(M1_DIR2, LOW);

               digitalWrite(M2_PWM, HIGH);
               digitalWrite(M2_DIR1, LOW);
               digitalWrite(M2_DIR2, LOW);
           }
       }

       if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
       {
           if (!is_punch_time)
           {
               punch_time_time = millis();
               is_punch_time = true;
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

           else if (speed < 0 && !back)
           {
               analogWrite(M1_PWM, pwm);
               digitalWrite(M1_DIR1, LOW);
               digitalWrite(M1_DIR2, HIGH);

               analogWrite(M2_PWM, pwm);
               digitalWrite(M2_DIR1, LOW);
               digitalWrite(M2_DIR2, HIGH);
           }

           else
           {
               if (compute_steering > 0)
               {
                   analogWrite(M1_PWM, pwm);
                   digitalWrite(M1_DIR1, LOW);
                   digitalWrite(M1_DIR2, HIGH);

                   analogWrite(M2_PWM, pwm * 0.6);
                   digitalWrite(M2_DIR1, LOW);
                   digitalWrite(M2_DIR2, HIGH);
               }
               else if (compute_steering < 0)
               {
                   analogWrite(M1_PWM, pwm * 0.6);
                   digitalWrite(M1_DIR1, LOW);
                   digitalWrite(M1_DIR2, HIGH);

                   analogWrite(M2_PWM, pwm);
                   digitalWrite(M2_DIR1, LOW);
                   digitalWrite(M2_DIR2, HIGH);
               }
               else
               {
                   analogWrite(M1_PWM, pwm);
                   digitalWrite(M1_DIR1, LOW);
                   digitalWrite(M1_DIR2, HIGH);

                   analogWrite(M2_PWM, pwm);
                   digitalWrite(M2_DIR1, LOW);
                   digitalWrite(M2_DIR2, HIGH);
               }
           }
       }
       cur_speed = speed;
   }
}

bool drift = false;

bool forced_steering_right = false;
bool forced_steering_left = false;
unsigned long forced_steering_time = 0;

void line_tracing_hello(float speed = 0.8, float turn_speed = 0.2, float right_steering = 0.9, float left_steering = -0.9, int cnt_IR_max = 60)
{ // 기본주행
    // 후진은 위험한 상황이니까 전진보다 먼저 고려
    if (cnt_IR_R > cnt_IR_max)
    {
        // 후진
        if (ir_r_value <= detect_ir)
        {
            drift = true;
            compute_steering = right_steering;
            compute_speed = -0.3;
        }
        else
        {
            drift = false;
            cnt_IR_R = 0;
            forced_steering_time = millis();
            forced_steering_left = true;
        }
    }
    else if (cnt_IR_L > cnt_IR_max)
    {
        // 후진
        if (ir_l_value <= detect_ir)
        {
            drift = true;
            compute_steering = left_steering;
            compute_speed = -0.3;
        }
        else
        {
            drift = false;
            cnt_IR_L = 0;
            forced_steering_time = millis();
            forced_steering_right = true;
        }
    }
    else if (forced_steering_right)
    {
        if (millis() - forced_steering_time > 300)
        {
            forced_steering_right = false;
        }
        compute_steering = right_steering;
        compute_speed = turn_speed;
    }
    else if (forced_steering_left)
    {
        if (millis() - forced_steering_time > 300)
        {
            forced_steering_left = false;
        }
        compute_steering = left_steering;
        compute_speed = turn_speed;
    }
    else if (ir_r_value <= detect_ir)
    { // 오른쪽 차선이 검출된 경우
        compute_steering = -1;
        compute_speed = turn_speed;
        cnt_IR_L = 0;
        cnt_IR_R++;
    }
    else if (ir_l_value <= detect_ir)
    { //왼쪽 차선이 검출된 경우
        compute_steering = 1;
        compute_speed = turn_speed;
        cnt_IR_R = 0;
        cnt_IR_L++;
    }
    else if (ir_r_value >= detect_ir && ir_l_value >= detect_ir)
    { //차선이 검출되지 않을 경우 직진
        compute_steering = 0;
        compute_speed = speed;
        cnt_IR_R = 0;
        cnt_IR_L = 0;
    }
}

void line_tracing(float speed = 0.8, float turn_speed = 0.2, float right_steering = 0.9, float left_steering = -0.9, int cnt_IR_max = 60)
{ // 기본주행
    // 후진은 위험한 상황이니까 전진보다 먼저 고려
    if (cnt_IR_R > cnt_IR_max)
    {
        // 후진
        if (ir_r_value <= detect_ir)
        {
            drift = true;
            compute_steering = right_steering;
            compute_speed = -0.3;
        }
        else
        {
            drift = false;
            cnt_IR_R = 0;
        }
    }
    else if (cnt_IR_L > cnt_IR_max)
    {
        // 후진
        if (ir_l_value <= detect_ir)
        {
            drift = true;
            compute_steering = left_steering;
            compute_speed = -0.3;
        }
        else
        {
            drift = false;
            cnt_IR_L = 0;
        }
    }
    else if (ir_r_value <= detect_ir)
    { // 오른쪽 차선이 검출된 경우
        compute_steering = -1;
        compute_speed = turn_speed;
        cnt_IR_L = 0;
        cnt_IR_R++;
    }
    else if (ir_l_value <= detect_ir)
    { //왼쪽 차선이 검출된 경우
        compute_steering = 1;
        compute_speed = turn_speed;
        cnt_IR_R = 0;
        cnt_IR_L++;
    }
    else if (ir_r_value >= detect_ir && ir_l_value >= detect_ir)
    { //차선이 검출되지 않을 경우 직진
        compute_steering = 0;
        compute_speed = speed;
        cnt_IR_R = 0;
        cnt_IR_L = 0;
    }
}

int parallel()
{
    if (right > 1000 || compute_speed == 0)
    {
        // 오른쪽이 너무 멀리 있거나 정지 상태라면 판단할 수 없음 (== 평행)
        return 0;
    }
    else
    { // 일단 전진 기준

        if (prev_right - right > 1)
        {
            // /모양 이니까 왼쪽으로 꺾기
            return -1;
        }
        else if (prev_right - right < -1)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

int parallel_left(int distance)
{
    if (left > 200 || compute_speed == 0)
    {
        // 오른쪽이 너무 멀리 있거나 정지 상태라면 판단할 수 없음 (== 평행)
        return 0;
    }
    else
    { // 일단 전진 기준
        // int sign_speed = (compute_speed > 0) - (compute_speed < 0);

        if (left - distance > 5)
        { // 왼쪽으로 꺾기
            return -1;
        }
        else if (left - distance < -5)
        { // 오른쪽으로 꺾기
            return 1;
        }
        else
        {
            return 0;
        }
    }
}
int parallel_right(int distance)
{

    if (right > 1000 || compute_speed == 0)
    {
        return 0;
    }
    else
    { // 일단 전진 기준
        // int sign_speed = (compute_speed > 0) - (compute_speed < 0);

        if (right - distance > 5)
        { // 오른쪽으로 꺾기
            return 1;
        }
        else if (right - distance < -5)
        { // 왼쪽으로 꺾기
            return -1;
        }
        else
        {
            return 0;
        }
    }
}

void _start()
{
    if (center > center_stop)
    {
        line_tracing();
    }
    else
    {
        compute_steering = 0;
        compute_speed = 0;
    }
}

unsigned long right_change_time = 0;

bool parking_p_time_checker = false;
unsigned long macro_start_time;

int right_change = 0;
bool after_back_up = false;
bool after_parking = false;
bool after_finding_line = false;
bool after_escape = false;

void parking_p()
{

    if (abs(prev_right - right) > 150 && millis() - right_change_time > 500)
    {
        right_change += 1;
        right_change_time = millis();
    }

    if (!after_back_up)
    { //후진 전
        if (right_change >= 3)
        { //후진, 위치잡기
            if (!parking_p_time_checker)
            { // 첫 진입시 시간 체크
                parking_p_time_checker = true;
                macro_start_time = millis();
            }

            // delay 제거하고 millis() 로 대체
            if (millis() - macro_start_time < 1900)
            {
                compute_steering = 1;
                compute_speed = -0.5;
            }
            else if (millis() - macro_start_time < 2900)
            {
                compute_steering = -1;
                compute_speed = -0.5;
            }
            else if (millis() - macro_start_time < 3700)
            {
                compute_steering = 0.7;
                compute_speed = 0.3;
            }
            else if (millis() - macro_start_time < 3750)
            {
                compute_steering = 0;
                compute_speed = -0.3;
            }
            else
            { // 시간 다 지나면 그만하기
                after_back_up = true;
                parking_p_time_checker = false;
            }
        }
        else
        { //쭉 직진
            compute_steering = parallel_left(90) * 0.4;
            compute_speed = 0.8;
        }
    }
    else if (!after_parking)
    { //후진 후 주차
        if (center > 300)
        {
            if (!parking_p_time_checker)
            {
                macro_start_time = millis();
                parking_p_time_checker = true;
            }

            if (millis() - macro_start_time < 700)
            {
                compute_steering = 0;
                compute_speed = 0;
            }
            else
            {
                after_parking = true;
                parking_p_time_checker = false;
            }
        }
        else
        {
            compute_steering = parallel() * 0.5;
            compute_speed = -0.1;
        }
    }
    else if (!after_finding_line)
    { //주차 후
        if (ir_l_value <= detect_ir || ir_r_value <= detect_ir)
        {
            // speed=0.1, turn_speed=0.1, right_steering=0.7, left_steering=-0.7, cnt_IR_max=15
            if (!parking_p_time_checker)
            {
                macro_start_time = millis();
                parking_p_time_checker = true;
            }

            if (millis() - macro_start_time < 500)
            {
                compute_steering = -1;
                compute_speed = -0.5;
            }
            else
            {
                after_finding_line = true;
            }
        }
        else
        {
            compute_steering = -0.5;
            compute_speed = 0.2;
        }
    }
    else if (!after_escape)
    {
        if (left < side_detect && right < side_detect)
        {
            after_escape = true;
        }
        else
        {
            line_tracing(0.1, 0.1, 0.8, -0.8, 10);
        }
    }
    else
    {
        line_tracing(1, 0.3, 1, -1, 150);
    }
}

int left_change = 0;
unsigned long left_change_time = 0;
bool center_ok = false;
bool flag_R = false;
bool t_flag1 = false;

void parking_t1()
{
    // 1. 좌회전
    if (!center_ok && center > center_detect)
    {
        compute_steering = 0.2;
        compute_speed = 0.15;
    }
    else
    {
        center_ok = true;
        if (!t_flag1)
        {
            compute_steering = -0.9;
            compute_speed = 0.07;
            if (millis() - last_stop_line_time > 2000 && (ir_r_value <= detect_ir || ir_l_value <= detect_ir || center < 100))
            {
                if (ir_r_value <= detect_ir)
                {
                    cnt_IR_R = 41;
                    flag_R = true;
                }
                else if (center < 100)
                {
                    flag_R = true;
                    cnt_IR_R = 41;
                }

                line_tracing(0.1, 0.1, 1, -1, 30);
                compute_speed = 0.1 * ((compute_speed > 0) - (compute_speed < 0));
                t_flag1 = true;
            }
        }
        else
        {
            line_tracing(0.15, 0.1, 1, -1, 30);
            compute_speed = 0.07 * ((compute_speed > 0) - (compute_speed < 0));
        }
    }
}

void parking_t2()
{
    if ((millis() - last_stop_line_time) < 1500)
    {
        if (flag_R)
        {
            compute_steering = 0.3;
        }
        else
        {
            compute_steering = -0.3;
        }
        compute_speed = -0.3;
    }
    else
    {
        line_tracing(0.3, 0.1, 0.5, -0.5, 100);
    }
}

void parking_t3()
{
    if (abs(prev_left - left) > 150 && millis() - left_change_time > 100)
    {
        left_change++;
        left_change_time = millis();
    }

    if (left_change >= 2 && millis() - left_change_time > 2000)
    {
        state++;
    }
    else
    {
        compute_steering = 0.15 * parallel_right(90);
        compute_speed = -0.37;
    }
}

void parking_t4()
{
    if (millis() - last_stop_line_time < 700)
    {
        compute_speed = 0;
        compute_steering = 0;
    }
    else if (millis() - last_stop_line_time < 1700)
    {
        compute_speed = 0.5;
        compute_steering = 0.2 * parallel_right(100);
    }
    else
    {
        line_tracing(0.5, 0.5, 0.6, -0.6, 50);
    }
}

int obstacle_cnt = 0;

void obstacle()
{
    if (obstacle_cnt != 0 && center < center_stop && left < side_detect)
    {
        tone(SPEAKER_PIN, 392);
        compute_speed = 0;
        compute_steering = 0;
        SetSteering(0);
        SetSpeed(0);
    }
    else if (obstacle_cnt < 20 && center < center_detect-20 && ir_l_value > detect_ir)
    { // 장애물 발견 & 왼쪽 차선 안보임
        compute_steering = -1;
        compute_speed = 0.3;
        obstacle_cnt++;
    }
    else if (obstacle_cnt > 0 && obstacle_cnt < 600 && ir_l_value > detect_ir)
    { // 장애물 발견 이후 & 왼쪽 차선 안보임
        compute_steering = -1;
        compute_speed = 0.3;
        obstacle_cnt++;
    }
    else
    {
        line_tracing(0.4, 0.3, 0.9, -0.6, 50);
        if (obstacle_cnt >= 20 && obstacle_cnt <= 600)
        {
            obstacle_cnt++;
        }
    }
}

bool CheckStopLine()
{
    // 방금 전에 정지선을 지나 온 경우
    if (millis() - last_stop_line_time < 1700)
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
        cnt_IR_BOTH = 0;
        return true;
    }
    return false;
}

void intersection1()
{
    if (millis() - last_stop_line_time < 700)
    {
        compute_speed = 0;
        compute_steering = 0;
    }
    else
    {
        line_tracing(0.7, 0.3, 1, -1, 150);
    }
}

void intersection2()
{
    if (millis() - last_stop_line_time < 700)
    {
        compute_speed = 0;
        compute_steering = 0;
    }
    else
    {
        line_tracing(0.45, 0.3, 1, -1, 150);
    }
}

void auto_driving(int state)
{
    switch (state)
    {
    case 0: // 출발
        _start();
        break;
    case 1: // 평행주차
        parking_p();
        break;
    case 2: // 8자 주행 1
        intersection1();
        break;
    case 3: // 8자 주행 2
        intersection2();
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
    case 7: // T 주차 4
        parking_t4();
        break;
    case 8: // 버스 피하기
        obstacle();
        break;
    case 9: // 버스 피하기
        obstacle();
        break;
    case 10: // 버스 피하기
        obstacle();
        break;
    }
}

bool song_friend = false;
void we_are_all_friends()
{
    if (millis() - melody_t > 200)
    {
        tone(SPEAKER_PIN, melody_we_are_all_friends[melody_index], 150);
        melody_t = millis();
        melody_index++;
    }
    if (melody_index >= 264)
    {
        melody_index = 0;
        song_friend = false;
    }
}

void road_201()
{
    if (millis() - melody_t > 300)
    {
        tone(SPEAKER_PIN, melody_road_201[melody_index], 250);
        melody_t = millis();
        melody_index++;
    }
    if (melody_index >= 96)
    {
        melody_index = 0;
        song_friend = true;
    }
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

    pinMode(SPEAKER_PIN, OUTPUT);

    max_pwm = max_ai_pwm;
    min_pwm = min_ai_pwm;

    SetSteering(0);
    SetSpeed(0);
    state = 0;
}

int i = 0;
void loop()
{
    if (i % 10 == 0)
    {
        prev_center = center;
        prev_left = left;
        prev_right = right;

        center = GetDistance(FC_TRIG, FC_ECHO);
        left = GetDistance(L_TRIG, L_ECHO);
        right = GetDistance(R_TRIG, R_ECHO);
        i %= 10;
    }
    i++;

    compute_steering = cur_steering;
    compute_speed = cur_speed;

    ir_r_value = ir_sensing(IR_R);
    ir_l_value = ir_sensing(IR_L);

    if (CheckStopLine())
    {
        state += 1;
    }

    if (song_friend)
    {
        we_are_all_friends();
    }
    else
    {
        road_201();
    }

    auto_driving(state);

    if (!is_stop_time && !is_punch_time){
        SetSteering(compute_steering);
    }
    SetSpeed(compute_speed, drift);
}