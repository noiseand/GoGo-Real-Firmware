/////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////

IOPointer  MotorENPins [MotorCount]={  MTR1_EN, MTR2_EN, MTR3_EN, MTR4_EN};
IOPointer  MotorCWPins [MotorCount]={  MTR1_CW, MTR2_CW, MTR3_CW, MTR4_CW};
IOPointer  MotorCCPins [MotorCount]={  MTR1_CC, MTR2_CC, MTR3_CC, MTR4_CC};

unsigned int CMD_STATE;

int gbl_cur_cmd, gbl_cur_param, gbl_cur_ext, gbl_cur_ext_byte;
int gblBurstModeBits;
int gblBurstModeCounter=0;   // tracks which sensor is the current burst mode sensor

int1 gblSlowBurstMode=0;  // determinds which burst mode we're in (0=normal, 1=slow)
int1 gblSlowBurstModeTimerHasTicked=0;  // ticks every 1/72 sec (by timer0)


int gblCurSensorChannel;

int gblMotorMode=0b00000000;   // default to normal mode
int gblActiveMotors;
int gblMotorDir=0;
int gblMotorONOFF = 0;
int gblMtrDuty[MotorCount+1] = {0xff,0xff,0xff,0xff,0xff};  // Motor PWM Duty cycle
unsigned int gblTimer0Counter = MotorCount; // Motor duty cycle counter.
unsigned int gblDutyCycleFlag = 0; // used to find the next duty cycle in timer0
unsigned int gblCurrentDutyIndex = 0; // keeps track of the current duty cycle being used.

// These two variables are for the NEWIR, IR commands in Cricket Logo
// We replace the IR with the serial comm, of course.
unsigned char gblMostRecentlyReceivedByte;
int1 gblNewByteHasArrivedFlag = 0;


int1 gblLogoIsRunning = 0;     // flags if logo procedures are runing
int1 gblButtonPressed = 0;    // flags when the run button is pressed
int1 gblBtn1AlreadyPressed = 0;
unsigned int16 gblWaitCounter =0;  // used for the wait cmd in Logo vm

unsigned int16 gblTimer = 0;   // This is the timer for the TIMER and RESETT commands

int gblCmdTimeOut = 0; // counter to make sure the command state is not stuck somewhere

int gblUsbBuffer[USB_BUFFER_SIZE];
int gblUsbBufferPutIndex=0;
int gblUsbBufferGetIndex=0;
int gblUsbBufferIsFull=FALSE;

int HILOWHasArrivedFlag = 0;
int16 adressHILOW = 0;

char gblFlashBuffer[getenv("FLASH_ERASE_SIZE")]; // buffer for flash write operations
char gblFlashBufferPtr=0; // pointer with-in the flash buffer
int16 gblFlashBaseAddress; // where the flash buffer shuld be written to in the flash mem
int ttTimer0 = 0; 


int   gblStkPtr,   // ptr to the top of the data stack
gblErrFlag;  // error flag
unsigned int16 gblStack[STACK_SIZE];

int   gblInputStkPtr;   // ptr to the top of the procedure input stack
unsigned int16 gblInputStack[INPUT_STACK_SIZE];

int16 globalVariables[16]={0};

unsigned int16 gblRecordPtr; // pointer to the current location in the data eeprom
// it will be initiazlied to the most recent record location
// in init_variables()

unsigned int16  gblMemPtr,     // FLASH/EEPROM pointer
gblRWCount;    // Read/Write length

int16 gblLoopAddress=0;   // Stores the start address of a Loop
int16 gblRepeatCount=0;  // Tracks the progress of the repeat command

int1 gblONFORNeedsToFinish=0;  // flags when onfor is launched
// it causes fetchNextOpcode() to return
// an Off command the next time it is called