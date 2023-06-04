#include "Arduino.h"
#include "GenericProtocol.h"

    void hshk(void *data);
    void ack(void *data);
    void rst(void *data);
    void send(void *data);
    void conn(void *data);


    typedef enum
    {
       tick_ev
      ,rcvhs_ev
      ,rcvak_ev
      ,rcv_data_ev
      ,data_sent_ev
      ,rst_ev
    } events;

    char *rgEventTxt[6]=
    {
       "tick_ev"
      ,"rcvhs_ev"
      ,"rcvak_ev"
      ,"rcv_data_ev"
      ,"data_sent_ev"
      ,"rst_ev"
    };

    typedef enum
    {
       disc_st
      ,hshk_st
      ,idle_st
      ,sent_st
    }states;

    char *rgStateTxt[4]
    {
       "disc_st"
      ,"hshk_st"
      ,"idle_st"
      ,"sent_st"
    };

    #define TT StateTransitionType
    //              tick             rcvhs            rcvak            rcv_data         data_sent    rst               
    TT disc_std[6]={{disc_st,&hshk}, {hshk_st,&ack},  {idle_st,&conn}, {disc_st,&hshk}, {disc_st,0}, {disc_st, &rst}};
    TT hshk_std[6]={{disc_st,&hshk}, {disc_st,&ack},  {idle_st,&conn}, {disc_st,&hshk}, {disc_st,0}, {disc_st, &rst}};
    TT idle_std[6]={{idle_st,0},     {disc_st,&rst},  {idle_st,0},     {idle_st,&ack},  {sent_st,0}, {disc_st, &rst}};
    TT sent_std[6]={{sent_st,&send}, {disc_st,&rst},  {idle_st,0},     {sent_st,&ack},  {sent_st,0}, {disc_st, &rst}};


    StateTransitionType *transTable[4]=
    {
       disc_std
      ,hshk_std
      ,idle_std
      ,sent_std
    };

    FSM fsm(transTable, rgStateTxt, rgEventTxt);


GenericProtocol::GenericProtocol()
{
}

void GenericProtocol::start()
{
  if (monitorMode)
  {
    fsm.enqueue(rcvak_ev, (void*)this);
  }
  else
  {
    fsm.setTickTimer(100,(void*)this);
  }
}

void GenericProtocol::setMonitorMode(bool fOn)
{
  monitorMode=fOn;
}

void GenericProtocol::handler()
{
  fsm.processEvents();
}

void GenericProtocol::setSendMethod(void (*method)(byte *data, int len))
{
  sendMethod=method;
}

void GenericProtocol::sendFrame(FrameType &frame)
{
  int len=1; // frame_types 2 and 3 (ack and hshk)
  if (frame.hdr.frame_type==0)//normal data (hdr, length byte and data)
  {
    len=frame.hdr.len+2;
  }
  else if (frame.hdr.frame_type==1)//broadcast
  {
    len=frame.bcast.bcast_len+1;
  }
  (*sendMethod)(frame.data, len);
}

void hshk(void *data)
{
  GenericProtocol *gp=(GenericProtocol*)data;
  FrameType hshk;
  hshk.hdr.frame_type=3;
  hshk.hdr.ack_num=0;
  hshk.hdr.frame_num=0;
  hshk.hdr.len=0;
  gp->sendFrame(hshk);
  gp->ack_num=gp->frame_num=0;
  fsm.setTickTimer(gp->timeout*5,data);
}

void ack(void *data)
{
  GenericProtocol *gp=(GenericProtocol*)data;
  FrameType ackFrame;
  ackFrame.hdr.frame_type=2;
  ackFrame.hdr.ack_num=gp->ack_num;
  ackFrame.hdr.frame_num=gp->frame_num;
  ackFrame.hdr.len=0;
  gp->sendFrame(ackFrame);
}

void rst(void *data)
{
  GenericProtocol *gp=(GenericProtocol*)data;
  gp->connected=false;
  if (gp->onDisconnect)
  {
    (*gp->onDisconnect)();
  }
  hshk(data);
}


void send(void *data)
{
  GenericProtocol *gp=(GenericProtocol*)data;
  if (gp->prevFrame)
  {
    gp->prevFrame->hdr.ack_num=gp->ack_num;
    gp->sendFrame(*gp->prevFrame);
    fsm.setTickTimer(gp->timeout,data);
    fsm.enqueue(data_sent_ev,data);
  }
  else
  {
    fsm.enqueue(rcvak_ev,data);
  }
}

void conn(void *data)
{
  GenericProtocol *gp=(GenericProtocol*)data;
  if (gp->prevFrame)free(gp->prevFrame);
  if (gp->nextFrame)free(gp->nextFrame);
  gp->nextFrame=0;
  gp->prevFrame=0;
  gp->connected=true;
  if (gp->onConnect)
  {
    (*gp->onConnect)();
  }
  if (!gp->monitorMode)
  {
    ack(data);
  }
  fsm.setTickTimer(0,data);
}

void GenericProtocol::sendData(void *data, int len)
{
  FrameType *pf;
  
  byte frameLen=len+2;
  int frameType=0;

  if (monitorMode || !connected) return;
  
  if (prevFrame) 
  {
    if (nextFrame)
      return; // just drop the new frame - prev frame must be acked first
    nextFrame=(FrameType*)malloc(frameLen);
    nextFrame->hdr.frame_type=frameType;
    nextFrame->hdr.len=len;
    memcpy(nextFrame->hdr.hdr_data,data,len);
    return;
  }
  pf = (FrameType*)malloc(frameLen);
  pf->hdr.frame_type=frameType;
  frame_num=(frame_num+1) & 7;
  pf->hdr.frame_num=frame_num;
  pf->hdr.ack_num=ack_num;
  pf->hdr.len=len;
  memcpy(pf->hdr.hdr_data, data, len);
  sendFrame(*pf);
  prevFrame=pf;
  Serial.println("sending data sent event");
  fsm.enqueue(data_sent_ev,(void*)this);
  fsm.setTickTimer(timeout,(void*)this);
}

bool GenericProtocol::sendBroadcast(void *data, int len)
{
  if (prevFrame) return false;
  FrameType *pf;
  pf=(FrameType*)malloc(len+1);
  pf->hdr.frame_type=1;
  pf->bcast.bcast_len=len;
  memcpy(pf->bcast.bcast_data,data,len);
  sendFrame(*pf);
  free(pf);
  return true;
}

void GenericProtocol::processFrameAck()
{
  fsm.enqueue(rcvak_ev,(void*)this);
  if (prevFrame)
  {
    free(prevFrame);
    prevFrame=0;
    fsm.setTickTimer(0,(void*)this);
    if (nextFrame)
    {
      frame_num=(frame_num+1) & 7;
      nextFrame->hdr.frame_num=frame_num;
      nextFrame->hdr.ack_num=ack_num;
      sendFrame(*nextFrame);
      prevFrame=nextFrame;
      nextFrame=0;
      fsm.enqueue(data_sent_ev,(void*)this);
      fsm.setTickTimer(timeout,(void*)this);
    }
  }
}

void GenericProtocol::processRecv(void *data, int len)
{
  FrameType *pf=(FrameType*)data;
  if (monitorMode)
  {
    if (onReceive && pf->hdr.frame_type==0) // data
    {
      if (pf->hdr.frame_num == frame_num) return; // duplicate data
      frame_num=pf->hdr.frame_num;
      (*onReceive)(pf->hdr.hdr_data,pf->hdr.len);
    }
    else if (onBroadcast && pf->hdr.frame_type==1) // broadcast
    {
      (*onBroadcast)(pf->bcast.bcast_data,pf->bcast.bcast_len);
    }
    return;
  }
  if (pf->hdr.frame_type==3) // handshake
  {
    fsm.enqueue(rcvhs_ev,(void*)this);
  }
  else if (pf->hdr.frame_type==2) // ack
  {
    if (pf->hdr.ack_num==frame_num)
    {
      processFrameAck();
    }
  }
  else if (pf->hdr.frame_type==0) // data
  {
    if (pf->hdr.ack_num==frame_num)
    {
      processFrameAck();
    }
    byte expected_frame=(ack_num+1) & 7;
    if (pf->hdr.frame_num==expected_frame)
    {
      ack_num=expected_frame;
      (*onReceive)(pf->hdr.hdr_data,pf->hdr.len);
    }
    else if (pf->hdr.frame_num!=ack_num)
    {
      fsm.enqueue(rst_ev, (void*)this);
      return;
    }
    fsm.enqueue(rcv_data_ev,(void*)this);
  }
  else if (onBroadcast && pf->hdr.frame_type==1) // broadcast
  {
    (*onBroadcast)(pf->bcast.bcast_data,pf->bcast.bcast_len);
  }
}

void GenericProtocol::setOnBroadcast(void (*method)(byte *data,int len))
{
  onBroadcast=method;
}

void GenericProtocol::setOnReceive(void (*method)(byte *data,int len))
{
  onReceive=method;
}

void GenericProtocol::setOnDisconnect(void (*method)())
{
  onDisconnect=method;
}

void GenericProtocol::setOnConnect(void (*method)())
{
  onConnect=method;
}

void GenericProtocol::setTimeout(long interval)
{
  timeout=interval;
}

void GenericProtocol::setLogMethod(void (*method)(const char * msg))
{
  fsm.setLogMethod(method);
}
