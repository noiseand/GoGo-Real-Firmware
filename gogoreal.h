/////////////////////////////////////////////////////////////////
//  Function Declaration
/////////////////////////////////////////////////////////////////

void stkPush(unsigned int16 stackItem);
unsigned int16 stkPop(void);
void inputPush(unsigned int16 stackItem);
unsigned int16 inputPop(void);
void stkErro();
void sendBytes(unsigned int16 memPtr, unsigned int16 count);
unsigned int16 fetchNextOpcode();
void sendBytes(unsigned int16 memPtr, unsigned int16 count);
unsigned int16 fetchNextOpcode();
void evalOpcode(unsigned char opcode);
void setHigh(IOPointer Pin);
void setLow(IOPointer Pin);
int  readPin(IOPointer Pin);
void flashSetWordAddress(int16 address);
short getBit(int InByte, int BitNo);
void setBit(int *InByte, int BitNo);
void clearBit(int *InByte, int BitNo);
void flashWrite(int16 InByte);
void Ping(int Param);
void TalkToMotor(int MotorBits);
void MotorControl(int MotorCmd);
void SetMotorPower(int Power);
void ChangeMotorPower(int delta);
void sortMtrDuty();
void SetMotorMode(int motorMode); // normal or servo

void ENHigh(int groupNo);
void ENLow(int groupNo);

void MotorON(int MotorNo);
void MotorOFF(int MotorNo);
void MotorRD(int MotorNo);
void MotorThisWay(int MotorNo);
void MotorThatWay(int MotorNo);
void MotorCoast(int MotorNo);
void miscControl(int cur_param, int cur_ext, int cur_ext_byte);

void beep();

void SetBurstMode(int SensorBits, int Mode);
void DoSensorStuff();
unsigned int16 readSensor(int sensorNo);
long getSensorVal();
void switchAdcChannel(int channelNo);

void ProcessInput();
void ProcessRFInput();
void init_variables();
void intro();
void Halt();
void initBoard();

void timer2ISR();
void version();
