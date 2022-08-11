// Alive 시간 설정
unsigned long period;
unsigned long lastSendTime = 0;
const unsigned int aliveTime = 30; // 데이터 전송 주기 [ms]

int getData;

// 데이터1 핸들러
int modeGroup;
int mode;
int portNo;
int idx;

// 데이터2 핸들러
int remainMode;
int remainPort;
int remainidx;
int remainData;

// Tone
int tonePin = 0;
int toneMap[12][8] = {
    {33, 65, 131, 262, 523, 1046, 2093, 4186},
    {35, 69, 139, 277, 554, 1109, 2217, 4435},
    {37, 73, 147, 294, 587, 1175, 2349, 4699},
    {39, 78, 156, 311, 622, 1245, 2849, 4978},
    {41, 82, 165, 330, 659, 1319, 2637, 5274},
    {44, 87, 175, 349, 698, 1397, 2794, 5588},
    {46, 92, 185, 370, 740, 1480, 2960, 5920},
    {49, 98, 196, 392, 784, 1568, 3136, 6272},
    {52, 104, 208, 415, 831, 1661, 3322, 6645},
    {55, 110, 220, 440, 880, 1760, 3520, 7040},
    {58, 117, 233, 466, 932, 1865, 3729, 7459},
    {62, 123, 247, 494, 988, 1976, 3951, 7902}};

// version 1.1.1   2020.01.20   ---------------------------------------------
// 디지털 인풋 포트 선택 업데이트에서 전체 상시 업데이트로 수정
// ***************************************************************************
// ************************* COMMAND Mode & Protocol *************************
// ***************************************************************************
//    x : don't care
//    p : port
//    v : value
// ***************************************************************************
// Mode Code : Entry → HW → Deviece
//----------------------------------------------------------------------------------------------------------
//                                            Data 1       -   Data 2        description
//---------------------------------------------------
#define SET_GROUP_1	 0x80

#define SET_GROUP_11 0x80
#define SET_INIT_DEVICE 0x80     // 128  b100 00 000
#define SET_DIGITAL_OUT 0x81
#define SET_NO_TONE 0x82         // 131  b100 00 011
#define SET_BLUE_PW 0x87         // 130  b100 00 010    b0vvv vvvv X2    데이터 두개로 네자리 비번 전송 99 - 99 -> 9999(MAX)
#define SET_PORT_DISABLE 0x86

#define SET_GROUP_12 0x88
#define SET_ALL_SERVO_RUNTIME 0x88
#define SET_ALL_SERVO_RUNTIME_L 0x88
#define SET_ALL_SERVO_RUNTIME_H 0x89
#define SET_MOTOR_CURRENT 0x8A   // 모드 설정 저장용 값 (업데이트시 사용)
#define SET_MOTOR_CURRENT_A 0x8A // 엔트리에서 오는 값
#define SET_MOTOR_CURRENT_B 0x8B // 엔트리에서 오는 값
// 모터는 상시 토크를 계산 하여, a0(14), a(1)15 에 저장한다.
// 전류 값은 "모터속도 설정 블록"과 별도로 "전류요청 블록"으로 부터 요청을 받아
// 전송하며 a0(14), a(1)15핀을 "SET_MOTOR_CURRENT"로 설정한다.
// 리턴 핀 : 0(A motor)→a0(14), 1(B motor)→a1(15), SET_ANALOG_IN 모드 설정을 한다.

#define SET_GROUP_MOTOR_S 0x90
#define SET_MOTOR_SPEED_Free 0x90 // b100 10 xvp - b0 vvv vvvv    p : 0 = A motor, 1 = B motor, v : speed - 8bit (0 ~ 200)
#define SET_MOTOR_SPEED_Fast 0x94 // 방향을 따로 지정하는 경우 핸들링1에서 remainmode 로 저장

#define SET_GROUP_TONE_PWM 0x98
#define SET_TONE 0x98            // b100 11 1pp - b0 ooo nnnn   o : 8 octaves(3bit)  n : 12notes(4bit)
#define SET_PWM 0x9C             // 208  b110 10 xpp - b0 vvv vvvv   v : set pwm value (0~100)

//---------------------------------------------------
#define SET_GROUP_2 0xA0
#define SET_SERVO_POSITION 0xA0  // 192  b110 00 vpp - b0 vvv vvvv   v : set servo value (0~180)

//---------------------------------------------------
#define SET_GROUP_3 0xC0 // 192  b110 00 ---
#define SET_SERVO_SPEED 0xC0   // 200  b110 0v ppp - b0 vvv vvvv   v : set servo speed (MAX 1~254)
#define SET_SERVO_RUNTIME 0xD0   // 200  b110 0v ppp - b0 vvv vvvv   v : set servo speed (MAX 1~254)

//---------------------------------------------------
#define SET_GROUP_INPUT 0xE0  // 224  b111 00 ppp
#define SET_ANALOG_IN 0xE0    // 224  b111 00 ppp                 request analog input data
#define SET_ULTRASONIC 0xE8   // 240  b111 11 ppp - b0 xxx xppp   Data1 포트=trig, Data2 포트:echo, 거리 값은 Data2 포트로 리턴
#define SET_DIGITAL_IN 0xF0
#define SET_DIGITAL_IN_L 0xF0   // 232  b111 01 ppp
#define SET_DIGITAL_IN_H 0xF8   // 232  b111 01 ppp

// ***************************************************************************
// Mode Code : Deviece → HW
//----------------------------------------------------------------------------------------------------------
//                                     Data 1   -   Data 2        description
#define COM_ALIVE 0x80            // b10000 000
#define COM_INIT_DEVICE 0x81      // b10000 001
#define COM_PORT_DISABLED 0x82    // b10000 010
#define COM_BLUETOOTH_PW_OK 0x83  // b10000 011
#define COM_BLUETOOTH_PW_ERR 0x84 // b10000 100

#define GET_DIGITAL_IN 0x88   // b1000 1 ---
#define GET_DIGITAL_IN_L 0x88 // b1000 1 0dd   b0 ddd dddd
#define GET_DIGITAL_IN_H 0x8A // b1000 1 1dd   b0 ddd dddd
//#define GET_ANALOG_IN 0x90~0xF8    // b1aaa 0 vvv   b0 vvv vvvv   ( XOR 연산 필요 22.3.26 )
