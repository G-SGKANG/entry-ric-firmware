
#include <Servo.h>
#include <SoftwareSerial.h>
#include "l298.h"
#include "Entry_ric_firmware_var.h"

// **************************************************
// ***************  Digital Port Map  ***************
// **************************************************
//
// 8bit timer counter 5, 6
// 16bit timer counter
// ■ MAP from Sever to Device
//
#define Size_TotalPort 22
#define Size_UsedPort 14
#define Size_Port_D 16
#define Size_Port_A 8
#define Size_Servo 7
#define Size_PWM 2

//   HW Pin NO  (1st Data)       2  ~3   4  ~5  ~6   7   8  ~9 ~10 ~11  |  14 15 16 17 18 19 20 21  | 
//   Block                       2  MA   4   5   6   7   8   9  10  MB  |  a0 a1 a2 a3 a4 a5 a6 a7  |
//                               ↓  ↓    ↓   ↓   ↓   ↓   ↓   ↓   ↓   ↓  |   ↓  ↓  ↓  ↓  ↓  ↓  ↓  ↓  |
//   IN/OUT, ULTRASONIC, TON     0       1   2   3   4           5      |   ↓  ↓  ↓  ↓  ↓  ↓  ↓  ↓  |
//   PWM                                     0   1                      |   ↓  ↓  ↓  ↓  ↓  ↓  ↓  ↓  |
//   SERVO POSITION              0           1   2               3      |   ↓  ↓  ↓  ↓  ↓  ↓  ↓  ↓  |
//   SERVO SPEED                 0           1   2               3      |                           | 
//   MOTOR SPEED                     0                               1  |   ↓  ↓  ↓  ↓  ↓  ↓  ↓  ↓  |
//   ANALOG                                                             |   0  1  2  3  4  5  6  7  |
//
// ■ MAP HW -> Device
int map_IdxToPortNo_D[Size_Port_D] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19}; //{2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19};
int map_IdxToPortNo_A[Size_Port_A] = {14, 15, 16, 17, 18, 19, 20, 21}; // {14, 15, 16, 17, 18, 19, 20, 21};
int servo_IdxToPortNo[Size_Servo] = {2, 4, 5, 6, 7, 10, 16};                // 서버모터 제어 핀 {2, 4, 5, 6, 7, 10, 16}
int mapPWM_IdxToPortNo[Size_PWM] = {5, 6};
int mapM_S_IdxToPortNo[2] = {3, 11}; // 모터 스피드 [ idx --> portNo ] 0:3, 1:11
int mapM_D_IdxToPortNo[2] = {0, 0};  // 모터 방향   [ idx --> portNo ]

// ■ MAP HW control                0  1  2  3  4   5   6   7   8   9  10  11  12  13
int mapUsedPort[Size_UsedPort] = {2, 4, 5, 6, 7, 10, 14, 15, 16, 17, 18, 19, 20, 21}; // {2, 4, 5, 6, 7, 10, 14, 15, 16, 17, 18, 19, 20, 21}
//                          0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
int mapMotor_PortToIdx[] = {0, 0, 0, 0, 0, 1, 2, 0, 0, 0, 3, 1, 0, 0, 0, 1}; // DC 모터 [ 3:0, 11:1 ], DC 전류 [14:0, 15:1], 서보 모터  [ 2:0, 5:1, 6:2, 10:3 ]

// ■ MAP Device -> HW
//                            (0)(1) 2 (3) 4  5  6  7 (8)(9) 10 11 12 13 14 15 16 17 18 19
int mapDigial_PortNoToIdx[] = {0, 0, 0, 1, 2, 3, 4, 5, 6, 7,  0, 1, 2, 3, 0, 0, 4, 5, 6, 7}; // Entry-HW에 Digtal값 전송 시 idx { D2:0, D4:2, D5:3, D6:4, D7:5, D10:0, A2:4, A3:5, A4:6, A5:7 }
//                          (0)(1) 2 (3) 4  5  6  7 (8)(9) 10 11 12 13 14 15 16  17  18  19  20  21
int mapAnalog_PortNoIdx[] = {0, 0, 2, 0, 3, 4, 5, 6, 0, 0, 7, 0, 0, 0, 8, 9, 10, 11, 12, 13, 14, 15}; // Entry-HW에 Analog 값 전송 시 idx { D2:2, D4:3, D5:4, D6:5, D7:6, D10:7  | A14:8 ~ A21:15 }

// **************************************************
// ******************  변 수 선 언  ******************
// **************************************************

int digitalIn_old_L = 0;
int digitalIn_new_L = 0;
int digitalIn_old_H = 0;
int digitalIn_new_H = 0;

int portMode[Size_TotalPort];  // 모드 저장
int portValue[Size_TotalPort]; // 컨트롤 값

// 서보 모터
// 서보 할당 핀             0  1  ② 3  4  ⑤ ⑥  7  8  9 ⑩
Servo servo[Size_Servo];
float servoT_Start[Size_Servo] = {0, 0, 0, 0, 0, 0, 0};
float ServoT_Now[Size_Servo] = {0, 0, 0, 0, 0, 0, 0};
float servoT_Run[Size_Servo] = {1800, 1800, 1800, 1800, 1800, 1800, 1800}; // [ms]

float Pi = 3.14159;

float servoSpeed[Size_Servo] = {666.667, 666.667, 666.667, 666.667, 666.667, 666.667, 666.667};  // 0~180 1.8 초

int servoP_Start[Size_Servo] = {1500, 1500, 1500, 1500, 1500, 1500, 1500};
int servoP_Now[Size_Servo] = {1500, 1500, 1500, 1500, 1500, 1500, 1500};
int servoP_Target[Size_Servo] = {1500, 1500, 1500, 1500, 1500, 1500, 1500};
int servoP_Delta[Size_Servo] = {0, 0, 0, 0, 0, 0, 0};

// 모터 설정 :  모터 생성, 드라이버(L298) 제어핀,상수, 변수
Mechatro motor[2];
#define PWM_A 3
#define PWM_B 11
#define BRAKE_A 9
#define BRAKE_B 8
#define DIR_A 12
#define DIR_B 13
#define SEN_A 14
#define SEN_B 15

void set_Divice()
{
    motor[0].attach(PWM_A, DIR_A, BRAKE_A, SEN_A);
    motor[1].attach(PWM_B, DIR_B, BRAKE_B, SEN_B);
}
// const float Km[4] = {0, 34, 1700, 2000};       // Motor torque constant  ( Km [gf.cm/A] = torque / current, at stall state )
// const float currentStall[4] = {0, 10, 10, 10}; // Motor torque constant  ( Km [gf.cm/A] = torque / current, at stall state )
/* 4.5V, 1A조건
                    RPM     gf.cm
   모터뭉치        10,000      34
   구동기어뭉치       255    1700
   기어드모터         300    2000
*/

// 모터 초기화 : 정지, 전류센싱 해제
void init_Motor()
{
    // 모터 초기화 속도 초기화
    motor[0].speed(0); // 정지
    motor[1].speed(0); // 정지
    portValue[PWM_A] = 0;
    portValue[PWM_B] = 0;

    // 모터 전류 센싱 초기화 (disable)
    portMode[SEN_A] = SET_PORT_DISABLE;
    portMode[SEN_B] = SET_PORT_DISABLE;
    portValue[SEN_A] = 0;
    portValue[SEN_B] = 0;
}

// 서보 모터 초기화 : 서보 detach, 위치,속도값 초기화
void init_Servo()
{
    for (int idx = 0; idx < Size_Servo; idx++)
    {
        if (servo[idx].attached())
        {
            servo[idx].detach();
        }
    }
}

// 디바이스 초기화 : 모든 디지털, 아날로그 핀 인풋 설정, 모터 정지, 서보 detach
void init_Divice_Entry_Stop()
{
    noTone(tonePin);
    digitalIn_old_L = -1;
    digitalIn_old_H = -1;

    // 디지털 H 포트를 인풋으로 설정하지 않으면,
    // output 출력이 있었을 경우 상태가 남고 출력을 하고 있을 경우 그 값이 아날로그 입력으로 들어온다
    // 디지털 H 포트를 input 으로 설정하여 모든 출력을 초기화 시킨 후, 아날로그핀으로 설정한다.
    for (int idx = 0; idx < Size_Port_D; idx++)
    {
        portNo = map_IdxToPortNo_D[idx];
        pinMode(portNo, INPUT);
        portMode[portNo] = SET_DIGITAL_IN;
    }

    for (int idx = 0; idx < Size_Port_A; idx++)
    {
        portNo = map_IdxToPortNo_A[idx];
        // analogRead(portNo);
        portMode[portNo] = SET_ANALOG_IN; //
        portValue[portNo] = -1;  // 종료뒤 값을 한번 전송하기 위해 값을 임으로 변경
    }

    // 모터 초기화
    init_Motor();
    init_Servo();
}
