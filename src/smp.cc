#include <iostream>
#include "stdafx.h"
#include "smp.h"
using namespace std; 

/*****************
  Schunk Motion Protocol
  1. CAN-bus
  2. polling & handling threads
  3. all independent modules included in CAN-communication
  4. cmd lib.

 ******************/

// constructor

SMP::SMP(){
  // uint8_t status_init = 0xE1, error_init = 0x00, mode_init = 0x07; 
  int module_idx = 0; 
  // it seems that we don't need this...
  for(int32_t i = kIDStart; i <= kIDEnd; i++){
    module_[module_idx].set_id(i);
    module_[module_idx].set_error_code(0x00);
    module_[module_idx].set_mode(0x00); 
    module_[module_idx].set_position(0.0); 
    module_[module_idx].set_velocity(0.0); 
    module_[module_idx].set_acceleration(0.0);
    module_[module_idx].set_current(0.0); 
    module_idx++; 
  }
  can_bus_handle_ = NULL; 
  can_polling_thread_handle_ = NULL;
  event_handling_thread_handle_ = NULL;
  baud_rate_ = NTCAN_BAUD_1000; 
  for(int i = 0; i < kModuleNumber; i++)
    fragment_length_[i]=0; 
  InitializeCriticalSection(&critical_section_);
  can_bus_polling_ = false;
  event_handling_ = false;
}

SMP::~SMP(){
  DeleteCriticalSection(&critical_section_); 
}

// communication functions...

NTCAN_RESULT SMP::StartCANBusComm(){
  NTCAN_RESULT return_value; 
  int can_bus_id = 0; 
  uint32_t flags = 0; 
  int32_t tx_queue_size = kBufferLength; // number of msgs
  int32_t rx_queue_size = kBufferLength; 
  int32_t tx_time_out = 100; // ms
  int32_t rx_time_out = 100; 
  
  // ntcan api, open can bus
  return_value = canOpen(can_bus_id, flags, tx_queue_size, rx_queue_size, tx_time_out, rx_time_out,
      &can_bus_handle_);  
  // check can open successful
  if(return_value != NTCAN_SUCCESS){
    cout << "CAN bus open failed with error " << return_value << " ..." << endl;
    cout << "exit..." << endl;
    return NULL;
  }
  // ntcan api, set baud rate
  return_value = canSetBaudrate(can_bus_handle_, baud_rate_);
  if(return_value != NTCAN_SUCCESS){
    cout<<"CAN bus set baudrate failed with error " << return_value << " ..." << endl;
    canClose(can_bus_handle_);
    return NULL;    
  }
  // scan can bus and add modules
  return_value = SetMsgModules(1);
  if(return_value == NTCAN_SUCCESS)
    return NTCAN_SUCCESS;
  else
    return NULL;
}

// delete modules and close can bus
NTCAN_RESULT SMP::CloseCANBusComm(){
  NTCAN_RESULT  return_value; 
  return_value = SetMsgModules(0);
  if(return_value != NTCAN_SUCCESS)
    return NULL;
  return_value = canClose(can_bus_handle_); 
  if(return_value != NTCAN_SUCCESS){
    cout<<" can bus can not be closed..."<<endl; 
    return NULL; 
  }
  cout << "can bus closed successfully..." << endl;
  return NTCAN_SUCCESS; 
}

NTCAN_RESULT SMP::SetMsgModules(int add_delete_flag){
  NTCAN_RESULT return_value; 
  int32_t module_msg_id = 0;
  for(int32_t module_id = kIDStart; module_id <= kIDEnd; module_id++){
    // add master
    module_msg_id = module_id | kMsgMasterToSlave;
    // ntcan api, add or delete module
    if(add_delete_flag)
      return_value = canIdAdd(can_bus_handle_, module_msg_id);
    else
      return_value = canIdDelete(can_bus_handle_, module_msg_id);
    if(return_value != NTCAN_SUCCESS){
      if(add_delete_flag)
        cout << "add ";
      else
        cout << "delete "; 
      cout << "master module " << module_id << " to can bus failed with error " 
        << return_value << " ..." << endl;
      canClose(can_bus_handle_);
      return NULL;
    }
    // add slave
    module_msg_id = module_id | kMsgSlaveToMaster;
    // ntcan api, add or delete module
    if(add_delete_flag)
      return_value = canIdAdd(can_bus_handle_, module_msg_id);
    else
      return_value = canIdDelete(can_bus_handle_, module_msg_id);
    if(return_value != NTCAN_SUCCESS){
      if(add_delete_flag)
        cout << "add ";
      else
        cout << "delete "; 
      cout << "slave module " << module_id << " to can bus failed with error " 
        << return_value << " ..." << endl;
      return NULL;
    }
    // add error
    module_msg_id = module_id | kMsgError;
    // ntcan api, add module
    if(add_delete_flag)
      return_value = canIdAdd(can_bus_handle_, module_msg_id);
    else
      return_value = canIdDelete(can_bus_handle_, module_msg_id);
    if(return_value != NTCAN_SUCCESS){
      if(add_delete_flag)
        cout << "add ";
      else
        cout << "delete "; 
      cout << "error module " << module_id << " to can bus failed with error " 
        << return_value << " ..." << endl;
      return NULL;
    }
  }
  return NTCAN_SUCCESS;
}

void SMP::OpenThreads(){
  Enable(&can_bus_polling_);
  Enable(&event_handling_);
  can_polling_thread_handle_ = (HANDLE)_beginthreadex(NULL, 0, SMP::CANPollingThreadStart, 
      this, CREATE_SUSPENDED, NULL); 
  ResumeThread(can_polling_thread_handle_); 
  event_handling_thread_handle_ = (HANDLE)_beginthreadex(NULL, 0, SMP::EventHandlingThreadStart, 
      this, CREATE_SUSPENDED, NULL); 
  ResumeThread(event_handling_thread_handle_); 
}

void SMP::CloseThreads(){
  cout << "closing threads..." << endl;
  Disable(&can_bus_polling_);   // thread polling
  cout << "can bus polling disabled... " << can_bus_polling_ << endl;
  Disable(&event_handling_);  // threa handling  
  cout << "event handling disabled..." << event_handling_ << endl;
}

void SMP::Enable(bool* pt_variable){
  EnterCriticalSection(&critical_section_);
  *pt_variable = true;
  LeaveCriticalSection(&critical_section_);
}

void SMP::Disable(bool* pt_variable){
  EnterCriticalSection(&critical_section_);
  *pt_variable = false;
  LeaveCriticalSection(&critical_section_);
} 

// actual polling thread function
void SMP::CANPolling(){
  NTCAN_RESULT return_value; 
  int parser_return_value = 0; 
  int32_t length = kBufferLength;
  // ntcan api
  // need to enter critical session...
  cout << "entering can bus polling thread..." << endl;
  while(can_bus_polling_){
	length = kBufferLength;
    return_value = canTake(can_bus_handle_, polling_buffer_, &length); 
    if(return_value != NTCAN_SUCCESS){
      cout << "failed to read can bus... thread stop..." << endl;
      break;
    }
    if(length != 0)
		ProcessBufferMsg(polling_buffer_, length);
  }
  cout << "exiting can bus polling thread..." << endl;
}

// need to fix...
void SMP::EventHandling(){
  int module_idx = 5;
  cout << "entering event handling thread..." << endl;
  while(event_handling_){
    ErrorHandling(module_idx);
  }
  cout << "exiting event handling thread..." << endl;
}

int SMP::ProcessBufferMsg(CMSG* cmsg_buffer, const int length){
  // update 
  int module_idx = 0;
  int parse_return_value = 0;
  float module_pos = 0; 

  for(int msg_idx = 0; msg_idx < length; msg_idx++){
    if(cmsg_buffer[msg_idx].data[1] == kFragmentBegin || cmsg_buffer[msg_idx].data[1] == kFragmentMiddle || 
        cmsg_buffer[msg_idx].data[1] == kFragmentEnd){
      // if it is a fragment message, push into the buffer
      module_idx = cmsg_buffer[msg_idx].id & kGetID - kOffset; 
      // copy the msg into the buffer
      fragment_buffer_[module_idx][fragment_length_[module_idx]].id = cmsg_buffer[msg_idx].id;
      fragment_buffer_[module_idx][fragment_length_[module_idx]].len = cmsg_buffer[msg_idx].len;
      for(int i = 0; i < kModuleNumber; i++)
        fragment_buffer_[module_idx][fragment_length_[module_idx]].data[i] = cmsg_buffer[msg_idx].data[i];
      fragment_length_[module_idx]++;
      if(cmsg_buffer[msg_idx].data[1] == kFragmentEnd){
        // process fragment here, potentially seperate to another process
        parse_return_value = ParseFragmentMsg(module_idx);        //  PARSE
        fragment_length_[module_idx] = 0;
        return parse_return_value;
      }
    }
    else{
      // non fragmented messages
      module_idx = cmsg_buffer[msg_idx].id & kGetID - kOffset;
      if(cmsg_buffer[msg_idx].data[1] == kPositionReached){ // cmd pos reached
        // position parsed
        module_[module_idx].set_status_code(module_[module_idx].status_code() | kPositionReachedTrue);  
        position_bytes_[0] = cmsg_buffer[msg_idx].data[2];
        position_bytes_[1] = cmsg_buffer[msg_idx].data[3];
        position_bytes_[2] = cmsg_buffer[msg_idx].data[4];
        position_bytes_[3] = cmsg_buffer[msg_idx].data[5];
        module_[module_idx].set_position(BytesToFloat(position_bytes_));
      }
      else if(cmsg_buffer[msg_idx].data[1] == kMoveEnd){
        // response from slave is position (standing still)
        module_[module_idx].set_status_code(module_[module_idx].status_code() | kMoveEndTrue & kMovingFalse); 
        position_bytes_[0] = cmsg_buffer[msg_idx].data[2];
        position_bytes_[1] = cmsg_buffer[msg_idx].data[3];
        position_bytes_[2] = cmsg_buffer[msg_idx].data[4];
        position_bytes_[3] = cmsg_buffer[msg_idx].data[5];
        module_[module_idx].set_position(BytesToFloat(position_bytes_));
      }
      else if(cmsg_buffer[msg_idx].data[1] == kMovePosition || 
          cmsg_buffer[msg_idx].data[1] == kMovePositionRelative || 
          cmsg_buffer[msg_idx].data[1] == kMovePositionTime || 
          cmsg_buffer[msg_idx].data[1] == kMovePositionTimeRelative){
        //response from Slave is estimated time 
        module_[module_idx].set_status_code(module_[module_idx].status_code() | kMovingTrue & kBrakeFalse); 
        time_bytes_[0] = cmsg_buffer[msg_idx].data[2];
        time_bytes_[1] = cmsg_buffer[msg_idx].data[3];
        time_bytes_[2] = cmsg_buffer[msg_idx].data[4];
        time_bytes_[3] = cmsg_buffer[msg_idx].data[5];
        module_[module_idx].set_time_required(BytesToFloat(time_bytes_));
      }
      else if(cmsg_buffer[msg_idx].data[1] == kMoveVelocity){
        // response from Slave is "OK". 
        if(cmsg_buffer[msg_idx].data[2] == kOK1 && cmsg_buffer[msg_idx].data[3] == kOK2)
          module_[module_idx].set_status_code(module_[module_idx].status_code() | kMovingTrue & kBrakeFalse); 
        else
          // need to be further coded...
          cout << "move velocity error... command will not be executed..." << endl;
      }
      else if(cmsg_buffer[msg_idx].data[1] == kStop || cmsg_buffer[msg_idx].data[1] == kEmergencyStop){
        // response from Slave is "OK" (ifSUCCESS)
        if(cmsg_buffer[msg_idx].data[2] == kOK1 && cmsg_buffer[msg_idx].data[3] == kOK2)
          module_[module_idx].set_status_code(module_[module_idx].status_code() | kBrakeTrue & kMovingFalse); 
        else
          cout << "stop fail... command will not be executed..." << endl;
      }
      else if(cmsg_buffer[msg_idx].data[1] == kError){
        module_[module_idx].set_status_code(module_[module_idx].status_code() | kErrorTrue); 
        module_[module_idx].set_error_code(cmsg_buffer[msg_idx].data[2]);
        // need to handle error here... or in the event handling thread...
      }
      else if(cmsg_buffer[msg_idx].data[1] == kAcknowledge){
        // response from Slave is "OK" (If all errors acknowledged)
        if(cmsg_buffer[msg_idx].data[2] == kOK1 && cmsg_buffer[msg_idx].data[3] == kOK2)
          module_[module_idx].set_status_code(module_[module_idx].status_code() | kErrorFalse); 
        else
          cout << "acknowledge failed..." << endl;
      }
      else if(cmsg_buffer[msg_idx].data[1] == kReference){
        // response from Slave is "OK" (If SUCCESS)
        if(cmsg_buffer[msg_idx].data[2] == kOK1 && cmsg_buffer[msg_idx].data[3] == kOK2)
          module_[module_idx].set_status_code(module_[module_idx].status_code() | kMovingTrue 
              | kReferencedTrue | kBrakeTrue);
        else
          cout << "reference failed..." << endl; 
      }
      else{
        cout << "unknown message..." << endl;
      }
    }
  }
  // return value need to be recoded...
  return 1;
}

int SMP::ParseFragmentMsg(int module_idx){
  // three fragment message...
  if(fragment_length_[module_idx] == 3 && fragment_buffer_[module_idx][0].data[2] == kGetState){
    position_bytes_[0] = fragment_buffer_[module_idx][0].data[3];
    position_bytes_[1] = fragment_buffer_[module_idx][0].data[4];
    position_bytes_[2] = fragment_buffer_[module_idx][0].data[5];
    position_bytes_[3] = fragment_buffer_[module_idx][0].data[6];
    module_[module_idx].set_position(BytesToFloat(position_bytes_)); 
    velocity_bytes_[0] = fragment_buffer_[module_idx][0].data[7];
    velocity_bytes_[1] = fragment_buffer_[module_idx][1].data[2];
    velocity_bytes_[2] = fragment_buffer_[module_idx][1].data[3];
    velocity_bytes_[3] = fragment_buffer_[module_idx][1].data[4];
    module_[module_idx].set_velocity(BytesToFloat(velocity_bytes_)); 
    current_bytes_[0] = fragment_buffer_[module_idx][1].data[5];
    current_bytes_[1] = fragment_buffer_[module_idx][1].data[6];
    current_bytes_[2] = fragment_buffer_[module_idx][1].data[7];
    current_bytes_[3] = fragment_buffer_[module_idx][2].data[2];
    module_[module_idx].set_current(BytesToFloat(current_bytes_));
    module_[module_idx].set_status_code(fragment_buffer_[module_idx][2].data[3]); 
    module_[module_idx].set_error_code(fragment_buffer_[module_idx][2].data[4]); 
    return 1;
  }
  else if(fragment_length_[module_idx] == 2 && fragment_buffer_[module_idx][0].data[2] == kGetState){
    return 1;
  }
  else if(fragment_length_[module_idx] == 1 && fragment_buffer_[module_idx][0].data[2] == kGetState){
    return 1;
  }
  else if(fragment_buffer_[module_idx][0].data[2] == kGetConfig){
    return 1;
  }
  else{
    // unknown message
    return 0;
  }
}


// command functions...

NTCAN_RESULT SMP::GetState(int index, float time_interval, uint8_t mode){
  // none-param : response with pos, vel, cur;  foronce;  
  // time: response with vel, acc, cur; constantly;   
  // time, mode:  mode = 0x01(pos), 0x02(vel), 0x04(cur); constantly; 
  NTCAN_RESULT return_value;
  CMSG cmsg;
  int32_t length = 1;
  // set all data to 0
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0;
  cmsg.id = module_[index].id() | kMsgMasterToSlave;
  if(time_interval == 0){
    cmsg.len = 2;   
    cmsg.data[0] = 0x01; 
    cmsg.data[1] = kGetState; //0x95
  }
  else{
    cmsg.len = 7;
    FloatToBytes(time_interval, time_bytes_);
    cmsg.data[0] = 0x06; 
    cmsg.data[1] = kGetState; 
    for(int i = 2; i < 6; i++)
      cmsg.data[i] = time_bytes_[i - 2];   
    cmsg.data[6] = mode;
    module_[index].set_mode(mode); 
  } 
  // ntcan api, send message
  return_value = canSend(can_bus_handle_, &cmsg, &length);
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send get state command..." << return_value << endl;
  return return_value;
}

NTCAN_RESULT SMP::Reference(int index){
  // Caution: need to know zero position of each module 
  NTCAN_RESULT return_value;
  CMSG cmsg;
  int32_t length = 1;
  return_value = Acknowledge(index);//CMD ACK first 
  if(return_value != NTCAN_SUCCESS){
    cout << "failed to acknowledge..." << endl;
    cout << "failed to reference..." << endl;
    return return_value;
  }
  cmsg.len = 2;
  cmsg.id = module_[index].id() | kMsgMasterToSlave;
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0;
  cmsg.data[0] = 0x01;
  cmsg.data[1] = kReference;
  // ntcan api, send message
  return_value = canSend(can_bus_handle_, &cmsg, &length); // reference
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send reference command..." << endl;
  // wait for 20 seconds before return...
  Sleep(20000);  // replaced with WaitForSingleObject(can_bus_handle_, time)
  return return_value;
}

NTCAN_RESULT SMP::Stop(int index){
  NTCAN_RESULT return_value;
  CMSG cmsg;
  int32_t length = 1;
  cmsg.len = 2;
  cmsg.id = module_[index].id() | kMsgMasterToSlave;
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0;
  cmsg.data[0] = 0x01;
  cmsg.data[1] = kStop;
  return_value = canSend(can_bus_handle_, &cmsg, &length); 
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send command stop..." << endl;
  return return_value;
}

NTCAN_RESULT SMP::EmergencyStop(int index){
  NTCAN_RESULT return_value;
  CMSG cmsg;
  int32_t length = 1;
  cmsg.len = 2;
  cmsg.id = module_[index].id() | kMsgMasterToSlave;
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0;
  cmsg.data[0] = 0x01;
  cmsg.data[1] = kEmergencyStop;
  return_value = canSend(can_bus_handle_, &cmsg, &length);
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send command emergency stop..." << endl;
  return return_value;
}

NTCAN_RESULT SMP::MovePosition(int index, float position){
  NTCAN_RESULT return_value;
  CMSG cmsg;
  int32_t length = 1;
  FloatToBytes(position, position_bytes_);
  cmsg.len = 6;
  cmsg.id = module_[index].id() | kMsgMasterToSlave;
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0;
  cmsg.data[0] = 0x05;
  cmsg.data[1] = kMovePosition;
  for(int i = 2;i < 6; i++)
    cmsg.data[i] = position_bytes_[i - 2];
  // ntcan api, send msg...
  return_value = canSend(can_bus_handle_, &cmsg, &length);
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to move position command..." << endl;
  return return_value;
}

NTCAN_RESULT SMP::MovePosition(int index, float position, float velocity, float acceleration, float current){
  // parameter: index, pos, {vel, acc, cur} 
  // this->checkStatus(index); 
  // this->checkPos(index, pos); 
  // vel = this->checkVel(index, vel); 
  // acc = this->checkAcc(index, acc); 
  // cur = this->checkCur(index, cur);
  NTCAN_RESULT return_value;
  CMSG cmsg[3];
  int32_t length = 3;
  FloatToBytes(position, position_bytes_);
  FloatToBytes(velocity, velocity_bytes_);
  FloatToBytes(acceleration, acceleration_bytes_);
  FloatToBytes(current, current_bytes_);
  for(int i = 0; i < kModuleNumber; i++){
    cmsg[0].data[i] = 0;
    cmsg[1].data[i] = 0;
    cmsg[2].data[i] = 0;
  }
  // assign values to msg body
  cmsg[0].id = module_[index].id() | kMsgMasterToSlave;
  cmsg[1].id = module_[index].id() | kMsgMasterToSlave;
  cmsg[2].id = module_[index].id() | kMsgMasterToSlave;
  cmsg[0].len = 8;
  cmsg[1].len = 8;
  cmsg[2].len = 7;
  cmsg[0].data[0] = 0x11; // length of each message data
  cmsg[1].data[0] = 0x0B;
  cmsg[2].data[0] = 0x05;
  cmsg[0].data[1] = kFragmentBegin; // fragmentation
  cmsg[1].data[1] = kFragmentMiddle;
  cmsg[2].data[1] = kFragmentEnd;
  cmsg[0].data[2] = kMovePosition; // command
  for(int i = 3; i < 7; i++)
    cmsg[0].data[i] = position_bytes_[i - 3];
  cmsg[0].data[7] = velocity_bytes_[0];
  for(int i = 2; i < 5; i++)
    cmsg[1].data[i] = velocity_bytes_[i - 1];
  for(int i = 5; i < 8; i++)
    cmsg[1].data[i] = acceleration_bytes_[i - 5];
  cmsg[2].data[2] = acceleration_bytes_[3];
  for(int i = 3; i < 7; i++)
    cmsg[2].data[i] = current_bytes_[i - 3];
  return_value = canSend(can_bus_handle_, cmsg, &length);
  if(return_value != NTCAN_SUCCESS)
    cout <<"failed to send move position command with multiple parameters..." << endl; 
  return return_value;
}

NTCAN_RESULT SMP::MoveVelocity(int index, float velocity, float current){
  // this->checkStatus(index);
  // vel = this->checkVel(index,vel);
  // cur = this->checkCur(index,cur);
  NTCAN_RESULT return_value;
  CMSG cmsg[2];
  int32_t length = 2;
  FloatToBytes(velocity, velocity_bytes_);
  FloatToBytes(current, current_bytes_);
  for(int i = 0; i < kModuleNumber; i++){
    cmsg[0].data[i] = 0;
    cmsg[1].data[i] = 0;
  }
  cmsg[0].id = module_[index].id() | kMsgMasterToSlave;
  cmsg[1].id = module_[index].id() | kMsgMasterToSlave;
  cmsg[0].len = 8;
  cmsg[1].len = 5;
  cmsg[0].data[0] = 0x09; 
  cmsg[1].data[0] = 0x03;
  cmsg[0].data[1] = kFragmentBegin; // fragmentation
  cmsg[1].data[1] = kFragmentEnd;
  cmsg[0].data[2] = kMoveVelocity; // command
  for(int i = 3; i < 7; i++)
    cmsg[0].data[i] = velocity_bytes_[i - 3];
  cmsg[0].data[7] = current_bytes_[0];
  for(int i = 2; i < 5; i++)
    cmsg[1].data[i] = current_bytes_[i - 1]; 
  return_value = canSend(can_bus_handle_, cmsg, &length); 
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send move velocity command..." << endl;
  return return_value;
}

NTCAN_RESULT SMP::SetTargetPosition(int index, float position){
  // val = this->checkPos(index, val); 
  NTCAN_RESULT return_value;
  CMSG cmsg; 
  int32_t length = 1; 
  FloatToBytes(position, position_bytes_); 
  cmsg.len = 6; 
  cmsg.id = module_[index].id() | kMsgMasterToSlave; 
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0; 
  cmsg.data[0] = 0x05; 
  cmsg.data[1] = kSetTargetPosition; 
  for(int i = 2; i < 6; i++)
    cmsg.data[i] = position_bytes_[i - 2]; 
  return_value = canSend(can_bus_handle_, &cmsg, &length); 
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send set target position command..." << endl;
  return return_value; 
}

NTCAN_RESULT SMP::SetTargetPositionRelative(int index, float position){
  // val = this->checkRelPos(index, val); 
  NTCAN_RESULT return_value;
  CMSG cmsg; 
  int32_t length = 1; 
  FloatToBytes(position, position_bytes_); 
  cmsg.len = 6; 
  cmsg.id = module_[index].id() | kMsgMasterToSlave; 
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0; 
  cmsg.data[0] = 0x05; 
  cmsg.data[1] = kSetTargetPositionRelative; 
  for(int i = 2; i < 6; i++)
    cmsg.data[i] = position_bytes_[i-2]; 
  return_value = canSend(can_bus_handle_, &cmsg, &length); 
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send set target position relative command..." << endl;
  return return_value; 
}

NTCAN_RESULT SMP::SetTargetVelocity(int index, float velocity){
  // val = this->checkVel(index, val); 
  NTCAN_RESULT return_value;
  CMSG cmsg;
  int32_t length = 1;
  FloatToBytes(velocity, velocity_bytes_);
  cmsg.len = 6;
  cmsg.id = module_[index].id() | kMsgMasterToSlave;
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0;
  cmsg.data[0] = 0x05;
  cmsg.data[1] = kSetTargetVelocity;
  for(int i = 2; i < 6; i++)
    cmsg.data[i] = velocity_bytes_[i - 2];
  return_value = canSend(can_bus_handle_, &cmsg, &length);
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send set target velocity command..." << endl;
  return return_value; 
}   

NTCAN_RESULT SMP::Acknowledge(int index){
  NTCAN_RESULT return_value;
  CMSG cmsg; 
  int32_t length = 1; 
  cmsg.len = 2; 
  cmsg.id = module_[index].id() | kMsgMasterToSlave; 
  for(int i = 0; i < kModuleNumber; i++)
    cmsg.data[i] = 0; 
  cmsg.data[0] = 0x01; 
  cmsg.data[1] = kAcknowledge; 
  return_value = canSend(can_bus_handle_, &cmsg, &length); // acknowledgement
  if(return_value != NTCAN_SUCCESS)
    cout << "failed to send acknowledge command..." << endl;
  return return_value;
}

// other functions
void SMP::FloatToBytes(float float_value, uint8_t* bytes_array){
  union{ 
    uint8_t b[4]; 
    float value;
  }tmp;
  tmp.value = float_value;
  for(int i = 0; i < 4; i++)
    bytes_array[i] = tmp.b[i];
}

float SMP::BytesToFloat(uint8_t* bytes_array){
  union{ 
    uint8_t b[4]; 
    float float_value; 
  }tmp;
  for(int i = 0; i < 4; i++)
    tmp.b[i] = bytes_array[i];
  return tmp.float_value;
}

float SMP::GetPosition(int module_idx){
	return module_[module_idx].position();
}

float SMP::GetVelocity(int module_idx){
	return module_[module_idx].velocity();
}

float SMP::GetCurrent(int module_idx){
	return module_[module_idx].current();
}

float SMP::GetAcceleration(int module_idx){
	return module_[module_idx].acceleration();
}

void SMP::ErrorHandling(int index){
  uint8_t module_error; 
  int32_t module_id; 
  module_id = module_[index].id(); 
  module_error = module_[index].error_code();  
  // cout<<"module "<< module_id << " reports error "<< module_error <<endl; 
  
  switch(module_error){
    case kInfoBoot:
      cout << "error codes: 0x0001(INFO). Module is successfully booted !" << endl;
      break;
    case kInfoNoRights:
      cout << "Error code: 0x03(INFO). No rights to cmd !" << endl;
      break;
    case kUnknownCommand:
      cout << "Error code: 0x04(INFO). Unknown command !" << endl;
      Stop(index);
      break;
    case kInfoFailed:
      cout << "Error code: 0x05. Command failed!" << endl;
      Stop(index);
      break;
    case kInfoNotReferenced:     // not referenced
      cout << "Error code: 0x06(INFO). Not referenced!" << endl;
      Stop(index);
      // CMD_REFERENCE(index); 
      break;
    case kInfoSearchSineVector:
      cout << "Error code: 0x0007(INFO). Search sine vector!" << endl; 
      break; 
    case kInfoNoErrors:
      cout << "Error code: 0x0008(INFO). No Errors!" << endl;
      break;
    case kCommunicationError:
      cout << "Error code: 0x09. Error in communication !" << endl;
      CloseCANBusComm(); // ??
      break;
    case kInfoTimeOut:
      cout << "Error code: 0x10. Timeout in communication!" << endl;
      break;
    case kInfoWrongBaudRate:
      cout << "Error code: 0x16. Wrong Baudrate!" << endl;
      break;
    case kInfoCheckSum:
      cout << "Error code: 0x19. Checksum is incorrect " << endl;
      Stop(index);
      break;
    case kInfoMessageLength:
      cout << "Error code: 0x1D. D-Len does not match." << endl;
      Stop(index);
      break;
    case kInfoWrongParameter:
      cout << "Error code: 0x1E. Wrong parameter!" << endl;
      Stop(index);
      break;
    case kErrorWrongRampType:
      cout << "Error code: 0xC8. No valid motion profile!" << endl;;
      Stop(index); 
      break;
    case kErrorConfigMemory:
      cout << "Error code: 0xD2. Configuration range is incorrect!" << endl;
      Stop(index); 
      break;
    case kErrorSoftLow:
      cout << "Error code: 0xD5. Module exceeded software low limit!" << endl;
      Stop(index);
      break;
    case kErrorSoftHigh:
      cout << "Error code: 0xD6. Module exceeded software high limit!" << endl;
      Stop(index);
      break;
    case kErrorEmergencyStop:
      cout << "Error code: 0xD9. Emergency Stop!" << endl;
      break;
    case kErrorTow:           // A common err.
      cout << "Error code: 0xDA. Towing Error! Reducing load of module!" << endl;
      Stop(index);
      break;
    case kErrorTooFast:
      cout << "Error code: 0xE4. Too Fast. Reducing current of module!" << endl;
      Stop(index);
      break;
    case kErrorCommunication: 
      cout << "Error code: 0xDD. Module fails to commutate." << endl; 
      Stop(index);
      break; 
    case kErrorFragmentation:
      cout << "Error code: 0xDC. Error in fragmentation. Data packets lost!" << endl;
      Stop(index);
      break;
    case kErrorCurrent:
      cout << "Error code: 0xDE. Current too large! Reducing load of module!" << endl;
      Stop(index);
      break;
    case kErrorI2T:
      cout << "Error code: 0xDF. I2T Error!  Reducing load of module!" << endl;
      Stop(index);
      break;
    case kErrorInitialize:
      cout << "Error code: 0xE0. Module could not be initialized" << endl;
      Stop(index);
      break;
    case kErrorTempLow:
      cout << "Error code: 0x70. Temprature of module is too low!" << endl;
      Stop(index);
      break;
    case kErrorTempHigh:
      cout << "Error code: 0x71. Temprature of module is too high!" << endl;
      Stop(index);
      break;
    case kErrorLogicLow:
      cout << "Error code: 0x72. Logic voltage is too low!" << endl;
      Stop(index); 
      break;
    case kErrorLogicHigh:
      cout << "Error code: 0x73. Logic voltage is too high!" << endl;   
      Stop(index); 
      break;
    case kErrorMotorVoltageLow:
      cout << "Error code: 0x74. Motor voltage is too low!" << endl;
      Stop(index); 
      break;
    case kErrorMotorVoltageHigh:
      cout << "Error code: 0x75. Motor voltage is too high!" << endl;
      Stop(index);
      break;
    case kErrorCableBreak:
      cout << "Error code: 0x76. Cable is defective!" << endl;
      Stop(index);
      break;
	case 0x00:
	  break;
    default: 
      cout << "Error not defined yet" << endl; 
      Stop(index); 
  }
  // Acknowledge(index);
}
