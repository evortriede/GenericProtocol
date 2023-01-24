#ifndef GenericProtocol_h
#define GenericProtocol_h

#include "Arduino.h"
#include "fsm.h"

typedef union
{
  struct
  {
    unsigned int frame_type : 2;
    unsigned int ack_num    : 3;
    unsigned int frame_num  : 3;
    byte len;
    byte hdr_data[];
  } hdr;
  struct
  {
    unsigned int bcast_frame_type : 2;
    unsigned int bcast_len : 6;
    byte bcast_data[];
  } bcast;
  byte data[];
} FrameType;

class GenericProtocol
{

  public:

    GenericProtocol();
    void start();
    void setMonitorMode(bool fOn);
    void handler();
    void setOnBroadcast(void (*method)(byte *data,int len));
    void setOnReceive(void (*method)(byte *data,int len));
    void setOnDisconnect(void (*method)());
    void setOnConnect(void (*method)());
    void setSendMethod(void (*method)(byte *data, int len));
    void setTimeout(long interval);
    void sendData(void *data,int len);
    bool sendBroadcast(void *data, int len);
    void processRecv(void *data, int len);
    void setLogMethod(void (*method)(const char * msg));

    
   //private:
   
    long timeout=1000;
    
    bool monitorMode=false;
    byte ack_num=0;
    byte frame_num=0;
    void (*sendMethod)(byte *data, int len);
    void (*onDisconnect)()=0;
    void (*onConnect)()=0;
    void (*onReceive)(byte *data, int len);
    void (*onBroadcast)(byte *data, int len)=0;
    FrameType *prevFrame=0;
    FrameType *nextFrame=0;

    void sendFrame(FrameType &frame);
    void processFrameAck();
};

#endif