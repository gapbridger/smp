#include "ntcan.h"
#include <windows.h>
#include <process.h>
#include <iostream>
#include <vector>
#include "Module.h"

// command code list

const uint8_t kStop = 0x91;
const uint8_t kEmergencyStop = 0x90;
const uint8_t kReference = 0x92;
const uint8_t kReferenceHand = 0x97;
const uint8_t kMoveEnd = 0x93;
const uint8_t kPositionReached = 0x94;
const uint8_t kGetState = 0x95;
const uint8_t kGetDetailedErrorInfo = 0x96;
const uint8_t kSetConfig = 0x81;
const uint8_t kGetConfig = 0x80;
const uint8_t kCommandError = 0x88;
const uint8_t kCommandWarning = 0x89;
const uint8_t kCommandInfo = 0x8A;
const uint8_t kCommandAcknowledge = 0x8B;
const uint8_t kMovePosition = 0xB0;
const uint8_t kMovePositionTime = 0xB1;
const uint8_t kMoveCurrent = 0xB3;
const uint8_t kMoveVelocity = 0xB5;
const uint8_t kMoveGrip = 0xB7;
const uint8_t kMovePositionRelative = 0xB8;
const uint8_t kMovePositionTimeRelative = 0xB9;
const uint8_t kMovePositionLoop = 0xBA;
const uint8_t kMovePositionTimeLoop = 0xBB;
const uint8_t kMovePositionRelativeLoop = 0xBC;
const uint8_t kMovePositionTimeRelativeLoop = 0xBD;
const uint8_t kSetTargetVelocity = 0xA0;
const uint8_t kSetTargetAcceleration = 0xA1;
const uint8_t kSetTargetJerk = 0xA2;
const uint8_t kSetTargetCurrent = 0xA3;
const uint8_t kSetTargetTime = 0xA4;
const uint8_t kSetTargetPosition = 0xA6; // cmd in new version  
const uint8_t kSetTargetPositionRelative = 0xA7; // cmd in new version
const uint8_t kFragmentBegin = 0x84;
const uint8_t kFragmentMiddle = 0x85; 
const uint8_t kFragmentEnd = 0x86;

// error code list

const uint8_t kInfoBoot = 0x01;
const uint8_t kInfoNoRights = 0x03;
const uint8_t kUnknownCommand = 0x04;
const uint8_t kInfoFailed = 0x05;
const uint8_t kInfoNotReferenced = 0x06;
const uint8_t kInfoSearchSineVector = 0x07; // 0x0007
const uint8_t kInfoNoErrors = 0x08; // 0x0008
const uint8_t kCommunicatonError = 0x09;
const uint8_t kInfoTimeOuti = 0x10;
const uint8_t kInfoWrongBaudRate = 0x16;  
const uint8_t kInfoCheckSum = 0x19;
const uint8_t kInfoMessageLength = 0x1D;
const uint8_t kInfoWrongParameter = 0x1E;
const uint8_t kErrorWrongRampType = 0xC8;
const uint8_t kErrorConfigMemory = 0xD2;
const uint8_t kErrorProgramMemory = 0xD3;
const uint8_t kErrorInvalidPhrase = 0xD4;
const uint8_t kErrorSoftLow = 0xD5;
const uint8_t kErrorSoftHigh = 0xD6;
const uint8_t kErrorPressue = 0xD7;
const uint8_t kErrorService = 0xD8;
const uint8_t kErrorEmergencyStop = 0xD9;
const uint8_t kErrorTow = 0xDA; //  common 
const uint8_t kErrorTooFast = 0xE4;
const uint8_t kErrorVPC3 = 0xDB;
const uint8_t kErrorFragmentation = 0xDC;
const uint8_t kErrorCommunication = 0xDD;   
const uint8_t kErrorCurrent = 0xDE;
const uint8_t kErrorI2T = 0xDF;
const uint8_t kErrorInitialize = 0xE0;
const uint8_t kErrorInternal = 0xE1;
const uint8_t kErrorTempLow = 0x70;
const uint8_t kErrorTempHigh = 0x71;
const uint8_t kErrorLogicLow = 0x72;
const uint8_t kErrorLogicHigh = 0x73;
const uint8_t kErrorMotorVoltageLow = 0x74;
const uint8_t kErrorMotorVoltageHigh = 0x75;
const uint8_t kErrorCableBreak = 0x76; // common 
const uint8_t kInfoFreeSpace = 0x02;
const uint8_t kInfoProgramEnd = 0x1F;
const uint8_t kInfoTrigger = 0x40;
const uint8_t kInfoReady = 0x41;   
const uint8_t kInfoGUIConnected = 0x42;
const uint8_t kInfoGUIDisconnected = 0x43;
const uint8_t kInfoProgram = 0x44;
const uint8_t kErrorMotorTemperature = 0x78;
const uint8_t kErrorHardLow = 0xE2;
const uint8_t kErrorHardHigh = 0xE3;
const uint8_t kErrorMath = 0xEC;
const uint8_t kErrorOverShoot = 0x82;
const uint8_t kErrorResolverCheckFailed = 0xEB;
const uint8_t kInfoUnknownAxisIndex = 0x11;
const uint8_t kErrorHardwareVersion = 0x83;

// other constant

const uint16_t kMsgMasterToSlave = 0x0500; 
const uint16_t kMsgSlaveToMaster = 0x0700;
const uint16_t kMsgError = 0x3000;
const uint16_t kGetID = 0x00FF;
const uint8_t kEmptyQueue = 0xF0;
const uint8_t kUnknownMsg = 0xF1;
const uint8_t kOutOfQueue = 0xF2; 
const uint8_t kRxTimeOut = 0xF3; 
const uint8_t kParseSuccess = 0xF4;
const uint8_t kFragging = 0xF5; 
const uint8_t kReferenceFail = 0xF6;
const uint8_t kStopFail = 0xF7;
const uint8_t kAcknowledgeFail = 0xF8; 
const uint8_t kEventReached = 0xF9; 
const uint8_t kEventNull = 0xFA; 
const uint8_t kModuleNumber = 8;
const uint8_t kIDStart = 8;
const uint8_t kIDEnd = 15;
const uint8_t kOffset = 8;      // to be  modified 


extern  MODULE  module[MODULE_NUM]; 

class SMP
{
  public:
    SMP();      // Module_Initializaation; Open_Thread;   
    ~SMP();     // Close_Thread 
    // communication functions
    NTCAN_RESULT StartCANBusComm(); // start communication
    NTCAN_RESULT CloseCANBusComm(); // close communication
    NTCAN_RESULT SetMsgModule();
    // commands 
    NTCAN_RESULT GetState(int index, float time_interval, uint8_t mode);   // mode: 01-pos; 02-vel; 04-cur
    NTCAN_RESULT Reference(int index);
    NTCAN_RESULT Stop(int index);
    NTCAN_RESULT EmergencyStop(int index);
    NTCAN_RESULT Acknowledge(int index); 
    NTCAN_RESULT MovePosition(int index, float pos); 
    NTCAN_RESULT MovePosition(int index, float pos, float vel, float acc, float cur);
    NTCAN_RESULT MoveVelocity(int index, float vel, float cur); 
    NTCAN_RESULT SetTargetPosition(int index, float pos); 
    NTCAN_RESULT SetTargetPositionRelative(int index, float posRel); 
    NTCAN_RESULT SetTargetVelocity(int index, float vel); 
    NTCAN_RESULT DeleteModule(); 
    // high level functions
    void CanPolling(); 
    void EventHandling(); 
    int ProcessNextMsg(int count, CMSG* cmsg); // parse next message in the buffer
    int ParseFragmentMsg(int index);
    // statio thread starting function
    static unsigned __stdcall CanPollingThreadStart(void * p_this){
      SMP* p_smp = (SMP*)p_this;
      p_smp->CanPolling();
      return 1;
    }
    static unsigned __stdcall EventHandlingThreadStart(void * p_this){
      SMP* p_smp = (SMP*)p_this;
      p_smp->EventHandling();
      return 1;
    }
    // variables

    CRITICAL_SECTION critical_section_; 

  private: 
    Module module_[kModuleNumber];
    NTCAN_HANDLE can_bus_handle_; 
    HANDLE can_polling_thread_handle; 
    HANDLE event_handling_thread_handle;  
    uint32_t baud_rate_;
    uint8_t buffer_size_;
    CMSG* polling_buffer_;    // MSG Read from module; 
    CMSG** fragment_buffer_; 
    int* fragment_length_; // fragment length of each bin

    bool can_bus_active_;
    bool can_bus_polling_;
    bool event_handling_; 


    void      errorHandler(int index); 

    void EnableCanComm();
    void DisableCanComm();
    void EnableCanPoll();
    void DisableCanPoll();
    void EnableEventHandle(); 
    void DisableEventHandle(); 
    void Float2Bit(float float_num, uint8_t* bit_num);
    float Bit2Float(uint8_t* bit_num);

}; 
