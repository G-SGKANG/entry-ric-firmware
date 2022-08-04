// 펌웨어 버전
#define version 100 // 1.1.0

#include "Entry_ric_firmware.h"


// **********************************************************************************
void setup()
{
  Serial.begin(57600);
  Serial.flush();
  delay(30);
  set_Divice();
  init_Divice_Entry_Stop();
}

// **********************************************************************************
void loop()
{

  while (Serial.available())
  {
    getData = Serial.read();
    if (getData & B10000000)
    { // getData 1 처리
      data1Handling(getData);
    }
    else
    { // getData 2 처리
      if (remainMode)
      { // remain getData 유무 확인
        data2Handling(getData);
      }
    }
  }

  // 모터 커런트 값은 RMS 방식으로 계산하기 때문에 전송 주기마다 없데이트 하면
  // 딜레이가 생긴다.
  ServoPoistionUpdate();

  period = millis() - lastSendTime;
  if (period > aliveTime)
  {
    Serial.write(COM_ALIVE); // 데이터 변화 없을 때 Alive 신호 전송
    updatePort();
    lastSendTime = millis();
  }
}

// **********************************************************************************
void data1Handling(int data1)
{
  remainMode = 0; // Data1 noly setting
  modeGroup = data1 & B11100000;

  switch (modeGroup)
  {
  // Level 11111111111111111111111111
  case SET_GROUP_COMMAND:
    mode = data1 & B11111000;
    switch (mode)
    {
    // Level 222222222222222222222222
    case SET_COM_SYSEM: // b1000 00 xxx
      switch (data1)
      {
      case SET_INIT_DEVICE:
        init_Divice_Entry_Stop();
        Serial.write(COM_INIT_DEVICE);
        // Serial.write(COM_INIT_DEVICE);
        //   Serial.print("SET_INIT_DEVICE : ");
        //   Serial.println(DDRB);
        break;
      case SET_PORT_DISABLE: // 사용하지 않음
        // init_Divice_Entry_Run();
        // Serial.write(COM_PORT_DISABLED);
        // Serial.write(COM_PORT_DISABLED);
        //  Serial.print("init_Divice_Entry_Run : ");
        //  Serial.println(init_Divice_Entry_Run);
        break;
      case SET_BLUE_PW:
        // Serial.print("AT start");
        set_blue_pw();
        break;
      case SET_NO_TONE:
        noTone(tonePin);
        portMode[tonePin] = SET_PORT_DISABLE;
        portValue[tonePin] = 0;
        tonePin = 0;
        // Serial.print("SET_NO_TONE : ");
        // Serial.println(SET_NO_TONE);
        break;
      case SET_MOTOR_CURRENT_A:
        portMode[SEN_A] = SET_MOTOR_CURRENT;
        break;
      case SET_MOTOR_CURRENT_B:
        portMode[SEN_B] = SET_MOTOR_CURRENT;
        break;
      case SET_SERVO_RUNTIME:
        remainMode = data1; // Data2 처리 설정
        remainPort = 0;
        remainData = 0;
        break;
      }
      break;
    // Level 222222222222222222222222
    case SET_GROUP_MOTOR_S:
      // Motor Command
      remainMode = data1 & B11111100; // Data2 처리 설정
      remainidx = data1 & B1;
      remainPort = mapM_S_IdxToPortNo[remainidx];
      remainData = (data1 << 6) & B10000000;
      break;
    // Level 222222222222222222222222
    case SET_SERVO_SPEED_L: // Data2 처리 설정
    case SET_SERVO_SPEED_H: // Data2 처리 설정
      remainMode = SET_SERVO_SPEED;
      idx = data1 & B111;
      remainidx = idx;
      remainPort = servo_IdxToPortNo[idx];
      remainData = (data1 << 4) & B10000000;
      break;
    }
    break;
  // Level 11111111111111111111111111
  case SET_GROUP_D_OUT:
    idx = data1 & B00000111;
    mode = data1 & B11110000;
    switch (mode)
    {
    case SET_DIGITAL_OUT_L:
      portNo = map_IdxToPortNo_L[idx];
      break;
    case SET_DIGITAL_OUT_H:
      portNo = map_IdxToPortNo_H[idx];
      break;
    }
    if (portMode[portNo] != mode)
    {
      portMode[portNo] = mode;
      pinMode(portNo, OUTPUT);
    }
    portValue[portNo] = (data1 & B00001000) >> 3;
    digitalWrite(portNo, portValue[portNo]);
    // isOutputUpdated[portNo] = 1;
    // Serial.print("SET_DIGITAL_OUT : ");
    // Serial.println(mode);
    // Serial.print("portNo : ");
    // Serial.println(portNo);
    // Serial.print("Value : ");
    // Serial.println(portValue[portNo]);
    break;
  // Level 11111111111111111111111111
  case SET_GROUP_SERVO_PWM_TON:
    mode = data1 & B11111000;
    switch (mode)
    {
    case SET_SERVO_POSITION_L: // Data2 처리 설정
    case SET_SERVO_POSITION_H: // Data2 처리 설정
      remainMode = SET_SERVO_POSITION;
      idx = data1 & B111;
      remainidx = idx;
      remainPort = servo_IdxToPortNo[idx];
      remainData = (data1 << 4) & B10000000;
      // Serial.print(mode);
      // Serial.print(" idx:");
      // Serial.print(idx);
      // Serial.print(" remainPort:");
      // Serial.println(remainPort);
      break;
    case SET_PWM: // Data2 처리 설정
      remainMode = SET_PWM;
      idx = data1 & B11;
      remainPort = mapPWM_IdxToPortNo[idx];
      remainData = 0;
      break;
    case SET_TONE: // Data2 처리 설정
      remainMode = SET_TONE;
      idx = data1 & B111;
      remainPort = map_IdxToPortNo_L[idx];
      remainData = 0;
      if (tonePin != remainPort)
      { // 버저를 여러개 포트에서 사용할 때 기존 버저 닫음.
        noTone(tonePin);
      }
      tonePin = remainPort;
      break;
    }
    break;
  // Level 11111111111111111111111111
  case SET_GROUP_INPUT:
    idx = data1 & B00000111;
    mode = data1 & B11111000;
    switch (mode)
    {
    case SET_DIGITAL_IN_L:
      portNo = map_IdxToPortNo_L[idx];
      if (portMode[portNo] != SET_DIGITAL_IN_L)
      {
        portMode[portNo] = SET_DIGITAL_IN_L;
        pinMode(portNo, INPUT);
      }
      // Serial.print("SET_DIGITAL_IN_L [idx: ");
      // Serial.print(idx);
      // Serial.print("] [PortNo: ");
      // Serial.print(portNo);
      // Serial.println("]");
      break;
    case SET_DIGITAL_IN_H:
      portNo = map_IdxToPortNo_H[idx];
      if (portMode[portNo] != SET_DIGITAL_IN_H)
      {
        portMode[portNo] = SET_DIGITAL_IN_H;
        pinMode(portNo, INPUT);
      }
      // Serial.print("SET_DIGITAL_IN_H [idx: ");
      // Serial.print(idx);
      // Serial.print("] [PortNo: ");
      // Serial.print(portNo);
      // Serial.println("]");
      break;
    case SET_ANALOG_IN:
      portNo = map_IdxToPortNo_A[idx];
      portMode[portNo] = SET_ANALOG_IN;
      break;
    case SET_ULTRASONIC:
      remainMode = SET_ULTRASONIC;
      remainPort = map_IdxToPortNo_L[idx]; // Data2 에서 에코 포트 값 설정
      remainData = 0;
      /*
        //Serial.print("SET_ULTRASONIC Data1  idx: ");
        //Serial.print(idx);
        //Serial.print(" mode:");
        //Serial.print(remainMode);
        //Serial.print(" remainPort:");
        //Serial.println(remainPort);
      */
      break;
    }
    break;
  }
}

void data2Handling(int data2)
{
  /*-------------------------------------------------
    값 변경 시 처리, 선처리를 해야 포트 설정 후 값을 사용
    -------------------------------------------------*/
  portValue[remainPort] = remainData | data2;
  /*-------------------------------------------------
    모드 변경 시 초기화 설정 값 처리
    -------------------------------------------------*/
  switch (remainMode)
  {
  case SET_SERVO_RUNTIME:
    AServoT_Start = millis();
    AservoT_Run = data2 * 100; // 1초가 10로 입력됨 --> ms 로 변환
    isAServerControlMode = 0x0f;
    // Serial.println("SET_SERVO_RUNTIME");
    break;

  case SET_SERVO_POSITION:
    if (portMode[remainPort] != SET_SERVO_POSITION)
    {
      portMode[remainPort] = SET_SERVO_POSITION;
      servo[remainidx].attach(remainPort);
    }
    // ServoP_Start, ServoP_NOw 는  Target에 도착했을 때 값을 동기화 시킨다.
    // 그렇지 않으면 서보 동작 함수에서 과거 값을 사용할 수 있다.
    // 여기에서 아래구분을 사용하면
    
    // 서보 전체 동작 신호 수신 -->  서보 전체 동작 함수 실행 --> 서보 Target 값 순으로 실행 시
    //  servoP_Target = 100         servoP_Target = 100        servoP_Target = 50
    //   servoP_Start = 0            servoP_Start = 0          servoP_Start = 0
    //   servoP_Now = 100              servoP_Now = 0          servoP_Now = 0 
    //                    (시간이 0에 가까우므로 스타트 값으로 설정된다)
    servoP_Start[remainidx] = servoP_Now[remainidx];
    servoP_Target[remainidx] = int(6.667 * float(portValue[remainPort]) + 900);
    servoP_Delta[remainidx] = servoP_Target[remainidx] - servoP_Now[remainidx];
    servoT_Start[remainidx] = millis();
    servoT_Run[remainidx] = float(abs(float(servoP_Delta[remainidx]) / servoSpeed[remainidx]) * 1000);
    // Serial.println("SET_SERVO_POSITION");
    // Serial.print("Set[");
    // Serial.print(remainidx);
    // Serial.print("]");
    // Serial.print(" Now:");
    // Serial.print(servoP_Now[remainidx]);
    // Serial.print(" Start:");
    // Serial.print(servoP_Start[remainidx]);
    // Serial.print(" Target:");
    // Serial.print(servoP_Target[remainidx]);
    // Serial.print(" Delta:");
    // Serial.print(servoP_Delta[remainidx]);
    // Serial.print(" RunTime:");
    // Serial.println(servoT_Run[remainidx]);
    break;

  case SET_SERVO_SPEED:
    servoSpeed[remainidx] = 6.667 * float(portValue[remainPort]);
    // Serial.print("Speed[");
    // Serial.print(remainidx);
    // Serial.print("]");
    // Serial.println(servoSpeed[remainidx]);
    break;

  case SET_MOTOR_SPEED_Free:
    portValue[remainPort] -= 100;
    if (tonePin & portValue[remainPort]) // 모터값이 있고, tone 을 사용하면 끈다.
    {
      noTone(tonePin);
      tonePin = 0;
    }
    motor[remainidx].speed(portValue[remainPort]);
    // Serial.print("speed : [ ");
    // Serial.print(remainPort);
    // Serial.print("]: ");
    // Serial.println(portValue[remainPort]);
    break;

  case SET_MOTOR_SPEED_Fast: //준비해야함.
    // portValue[remainPort] -= 100;
    // int dirPortNO = mapM_D_IdxToPortNo[remainidx];
    // motor[remainidx].speed(portValue[remainPort], portValue[dirPortNO]);
    //  Serial.print("speed : [ ");
    //  Serial.print(portNo);
    //  Serial.print("]: ");
    //  Serial.println(remainData);
    break;

  case SET_PWM:
    if (portMode[remainPort] != SET_PWM)
    {
      portMode[remainPort] = SET_PWM;
      pinMode(remainPort, OUTPUT);
    }
    remainData = portValue[remainPort];
    remainData = map(remainData, 0, 100, 0, 255);
    analogWrite(remainPort, remainData);
    // Serial.print("SET_PWM [");
    // Serial.print(portNo);
    // Serial.print("] : ");
    // Serial.println(portValue[portNo]);
    break;

  case SET_TONE: //#define SET_TONE 0xD8  // b110 11 ppp - b0 ooo nnnn   o : 8 octaves(3bit)  n : 12notes(4bit)
    // 모터가 pwm을 사용하므로 모터를 정지시킨다.
    motor[0].speed(0); // 정지
    motor[1].speed(0); // 정지
    if (portMode[remainPort] != SET_TONE)
    {
      portMode[remainPort] = SET_TONE;
      pinMode(remainPort, INPUT); // 해당 핀을 이전에 PWM으로 사용한 경우 tone() 사용전 input 모드 설정하여 PWM설정 초기화
    }
    {
      int notes = portValue[remainPort] & 0x0F;
      int octaves = (portValue[remainPort] >> 4) & 0x07;
      tone(remainPort, toneMap[notes][octaves]);
    }
    break;

  case SET_ULTRASONIC:
    if (portMode[remainPort] != SET_ULTRASONIC)
    {
      int echo_pin = map_IdxToPortNo_L[data2];
      portMode[remainPort] = SET_ULTRASONIC;
      pinMode(remainPort, OUTPUT); // triger 핀
      pinMode(echo_pin, INPUT);    // echo 핀
      portValue[echo_pin] = 0;
    }
    // Serial.print("SET_ULTRASONIC Data2:  ");
    // Serial.print(" mode:");
    // Serial.print(remainMode);
    // Serial.print(" remainPort:");
    // Serial.println(remainPort);
    break;
  }
  remainMode = 0;
}

// **********************************************************************************
void updatePort()
{
  /*
     출력모드에 값이 업데이트 되었는지 확인 하고 1회만 업데이트 함
     출력모드는 엔트리에서 데이터가 오면 isOutputUpdated = 1 초기화 (엔트리는 값이 바뀔때만 값을 보낸다. 2회)
     입력모드 설정은 isOutputUpdated 값이 항상 1 초기화 이후 값을 변경하지 않음. 확인하여 이전값과 다르면 실행한다.
  */
  int value;
  digitalIn_new_L = 0;
  digitalIn_new_H = 0;

  for (int idx = 0; idx < Size_UsedPort; idx++)
  {
    portNo = mapUsedPort[idx];
    switch (portMode[portNo])
    {
    case SET_ANALOG_IN:
      value = analogRead(portNo);
      send_A_Value(portNo, value);
      break;

    case SET_DIGITAL_IN_L:
      value = digitalRead(portNo);
      digitalIn_new_L |= value << mapDigial_PortNoToIdx[portNo];
      // Serial.print("updateProt_D_IN_L [PortNo: ");
      // Serial.print(portNo);
      // Serial.print("] [idx: ");
      // Serial.print(mapDigial_PortNoToIdx[portNo]);
      // Serial.print("] [value: ");
      // Serial.print(value);
      // Serial.println("]: ");
      break;

    case SET_DIGITAL_IN_H:
      value = digitalRead(portNo);
      digitalIn_new_H |= value << mapDigial_PortNoToIdx[portNo];
      // Serial.print("updateProt_D_IN_H [PortNo: ");
      // Serial.print(portNo);
      // Serial.print("] [idx: ");
      // Serial.print(mapDigial_PortNoToIdx[portNo]);
      // Serial.print("] [value: ");
      // Serial.print(value);
      // Serial.println("]: ");
      break;

    case SET_ULTRASONIC:
    {
      unsigned long ultra_distance;
      int echoPortNo = map_IdxToPortNo_L[portValue[portNo]]; // 트리거 포트에 Value 에 저장된 값이 echo idx
      digitalWrite(portNo, LOW);                             // 트리거 Low
      delayMicroseconds(2);
      digitalWrite(portNo, HIGH); // 트리거 high
      delayMicroseconds(10);
      digitalWrite(portNo, LOW); // 트리거 Low
      ultra_distance = pulseIn(echoPortNo, HIGH, 29412);
      //  pulseIn함수의 단위는 us(마이크로 세컨드)
      //  5m 기준으로 timeout 시간을 지정한다.
      //  소리속도 340m/s
      //  Max 거리 5m
      //  timeout = 왕복시간
      //          = 왕복거리 5[m]*2 / 340[m/s] = 0.029412 [s]
      //          = 29412[us] = 29ms

      if (ultra_distance > 0)
      {
        ultra_distance = ultra_distance / 57;
        send_A_Value(echoPortNo, ultra_distance);
        // Serial.println("[ ULTRA ]");
        // Serial.print("trig: ");
        // Serial.print(portNo);
        // Serial.print("  echo:");
        // Serial.print(echoPortNo);
        // Serial.print(" distance:");
        // Serial.print(ultra_distance);
        // Serial.print(" portValue:");
        // Serial.print(portValue[echoPortNo]);
        // Serial.println("cm");
      }
    }
    break;

    case SET_MOTOR_CURRENT:
    {
      int ii = mapMotor_PortToIdx[portNo];         // 모터 인덱스
      value = (int)(motor[ii].getCurrent() * 0.1); // 전송 값이 10bit 초과로 [mA]->[cA] 변환, entryJS 블록에서 mA로 환원
      send_A_Value(portNo, value);
    }
    break;
    }
  }
  send_D_Value();
}
// **********************************************************************************
// 값이 현재 값과 다를 경우 전송한다.
void send_A_Value(int portNo, int sendData)
{
  if (sendData != portValue[portNo])
  {
    portValue[portNo] = sendData;
    int sendData1 = int(sendData >> 7);
    sendData1 = B10000000 | (mapAnalog_PortNoIdx[portNo] << 3) | sendData1;
    // Serial.print("send_A_Value - Mode:");
    // Serial.print(portMode[portNo]);
    // Serial.print(" [PortNO:");
    // Serial.print(portNo);
    // Serial.print("] [idx:");
    // Serial.print(mapAnalog_PortNoIdx[portNo]);
    // Serial.print("] [");
    // Serial.print(sendData);
    // Serial.println("]");
    Serial.write(sendData1);
    Serial.write(sendData & B01111111);
  }
}

void send_D_Value()
{
  // Serial.println("send_D_Value()");
  if (digitalIn_old_L != digitalIn_new_L)
  {
    // Serial.print("Get_D_New_L: ");
    // Serial.println(digitalIn_new_L, BIN);
    digitalIn_old_L = digitalIn_new_L;
    Serial.write(GET_DIGITAL_IN_L);
    Serial.write(digitalIn_new_L);
    Serial.write(GET_DIGITAL_IN_L);
    Serial.write(digitalIn_new_L);
  }
  if (digitalIn_new_H != digitalIn_old_H)
  {
    // Serial.print("Get_D_New_H: ");
    // Serial.println(digitalIn_new_H, BIN);
    digitalIn_old_H = digitalIn_new_H;
    Serial.write(GET_DIGITAL_IN_H);
    Serial.write(digitalIn_new_H);
    Serial.write(GET_DIGITAL_IN_H);
    Serial.write(digitalIn_new_H);
  }
}

void ServoPoistionUpdate()
{
  if (isAServerControlMode)
  {
    AServoT_Now = millis() - AServoT_Start;
    if (AServoT_Now >= AservoT_Run)
    {
      for (idx = 0; idx < Size_Servo; idx++)
      {
        servoP_Now[idx] = servoP_Target[idx];
        servoP_Start[idx] = servoP_Target[idx];
        servo[idx].writeMicroseconds(servoP_Now[idx]);
        isAServerControlMode ^= (0x01 << idx);
      }
    }
    else
    {
      AServoT_Now = 0.5 * (sin(Pi * (AServoT_Now / AservoT_Run - 0.5)) + 1.0);
      for (idx = 0; idx < Size_Servo; idx++)
      {
        servoP_Now[idx] = int(AServoT_Now * servoP_Delta[idx] + servoP_Start[idx]);
        servo[idx].writeMicroseconds(servoP_Now[idx]);
      }
    }
  }
  else
  {
    for (idx = 0; idx < Size_Servo; idx++)
    {
      portNo = servo_IdxToPortNo[idx];
      if (portMode[portNo] == SET_SERVO_POSITION)
      {
        if ((servoP_Now[idx] != servoP_Target[idx]))
        {
          ServoT_Now[idx] = millis() - servoT_Start[idx];
          if (ServoT_Now[idx] >= servoT_Run[idx])
          {
            servoP_Now[idx] = servoP_Target[idx];
            servoP_Start[idx] = servoP_Target[idx];
          }
          else
          {
            servoP_Now[idx] = int((ServoT_Now[idx] / servoT_Run[idx]) * servoP_Delta[idx] + servoP_Start[idx]);
          }
          servo[idx].writeMicroseconds(servoP_Now[idx]);
        }
      }
    }
  }
}

//---------------------------------------------------------------------------------
void set_blue_pw()
{

  SoftwareSerial blueSerial(4, 7); // RX, TX
  blueSerial.begin(38400);

  String ATCommand;
  int pswd;
  int pwCcomfirm = 0;
  int pwcheck[4] = {0, 0, 0, 0};

  delay(50);
  pswd = Serial.read();
  pswd = (pswd * 100) + Serial.read();

  if (pswd < 10)
  {
    ATCommand = "000" + (String)pswd;
  }
  else if (pswd < 100)
  {
    ATCommand = "00" + (String)pswd;
  }
  else if (pswd < 1000)
  {
    ATCommand = '0' + (String)pswd;
  }
  else
  {
    ATCommand = (String)pswd;
  }
  ATCommand = "AT+PSWD=" + ATCommand + "\r\n";

  // Serial.println(ATCommand);

  blueSerial.print("AT\r\n");
  delay(50);
  blueSerial.print("AT\r\n");
  delay(50);
  blueSerial.print("AT+ROLE=0\r\n");
  delay(50);
  blueSerial.print("AT+UART=57600,1,0\r\n");
  delay(50);
  blueSerial.print(ATCommand);
  delay(50);
  blueSerial.print("AT+CMODE=1\r\n");
  delay(50);
  blueSerial.print("AT+PSWD\r\n");
  delay(50);

  while (blueSerial.available())
  {
    if ((blueSerial.available() > 6) && (blueSerial.available() < 11))
    {
      pwcheck[0] = pwcheck[1];
      pwcheck[1] = pwcheck[2];
      pwcheck[2] = pwcheck[3];
      pwcheck[3] = blueSerial.read();
    }
    else
    {
      blueSerial.read();
    }
  }

  for (int i = 0; i < 4; i++)
  {
    pwCcomfirm = pwCcomfirm * 10 + (pwcheck[i] - 48);
  }

  if (pwCcomfirm == pswd)
  {
    // Serial.print("Success");
    Serial.write(COM_BLUETOOTH_PW_OK);
  }
  else
  {
    // Serial.print("Fail");
    Serial.write(COM_BLUETOOTH_PW_ERR);
  }

  blueSerial.end();
}
