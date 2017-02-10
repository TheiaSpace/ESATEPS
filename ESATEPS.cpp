/*
 * This file is part of Theia Space's ESAT EPS library.
 *
 * Theia Space's ESAT EPS library is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT EPS library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT EPS library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "ESATEPS.h"
#include <USBSerial.h>
#include <Wire.h>

bool ENABLED5=false;boolean ENABLED3=true;
boolean isOC2=false;
///MPPT
int min1=0;
int max1=255; //50;
int MPPTDelta1=1;int MPPTDelta2=1;
int PWMValue1=50;double previousI1=0;//int u1=0;
int PWMValue2=50;double previousI2=0;//int u1=0;
///////
void receiveEvent(int howMany){
EPS.command = Wire.read();
if (howMany>1){EPS.param=Wire.read();} else {EPS.param=0;}
}
void requestEvent(){
  Wire.write(EPS.bufferH,51);
}
byte ESATEPS::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire1.beginTransmission(Address);
  Wire1.write(Register);
  byte error=Wire1.endTransmission();
  if (error!=0){}
 else {
  // Read Nbytes
  Wire1.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  for (int index=0;index<Nbytes;index++) {
  Data[index]=Wire1.read();}
 }
 return error;
}

byte ESATEPS::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire1.beginTransmission(Address);
  Wire1.write(Register);
  Wire1.write(Data);
  byte error=Wire1.endTransmission();
  return error;
}

void ESATEPS::init() {
  unsigned char Id;
  Flash.read(flash, &Id, 1);
  myId=(int)Id;
  command=0;param=0;
  ACTIVEMPPT=true;MPPTMODE=true;MPPTFIXED=false;
  EPSStatus=0;
  pinMode(EN5V,OUTPUT);digitalWrite(EN5V,LOW);
  pinMode(EN3V,OUTPUT);digitalWrite(EN3V,HIGH);
  pinMode(OC1,INPUT_PULLUP);pinMode(OC2,INPUT_PULLUP);
  attachInterrupt(OC2, switch_fun, FALLING);
  Wire1.begin();
  Wire.begin(2);
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent); 
  analogReference(INTERNAL2V5);
  USB.begin();  
}

void ESATEPS::serialLog(String comment){
USB.println("{\"type\":\"log\",\"data\":\""+comment+"\"}");
}
void ESATEPS::handleCommand(){
 switch(EPS.command){
 case 0: 
 //set id and store in flash
 Flash.erase(flash); 
 myId=param;
 unsigned char p;p=param;
 Flash.write(flash, &p ,1);
 EPS.command=0;
 break;
 case 1:
   ENABLED5=!ENABLED5;
   if(ENABLED5){digitalWrite(EN5V,HIGH);} else {digitalWrite(EN5V,LOW);}
   EPS.command=0;
   break;
 case 2:
   ENABLED3=!ENABLED3;
   if(ENABLED3){digitalWrite(EN3V,HIGH);} else {digitalWrite(EN3V,LOW);}
   EPS.command=0;
   break;
 case 3:
   MPPTMODE=true;MPPTFIXED=false;
   MPPTDelta1=1;MPPTDelta2=1;
   PWMValue1=50;previousI1=0;PWMValue2=50;previousI2=0;
   EPS.command=0;
 break;
 case 4:
   MPPTMODE=false;MPPTFIXED=false;
   MPPTDelta1=1;MPPTDelta2=1;
   EPS.command=0;
 break;
 case 5:
   EPS.param=constrain(EPS.param,0,255);
   MPPTFIXED=true;PWMValue1=EPS.param;PWMValue2=EPS.param;
   analogWrite(PWM1, PWMValue1);analogWrite(PWM2, PWMValue2);
   EPS.command=0;
 break;
 default:
   EPS.command=0;
 break; 
}
}

String ESATEPS::build_tm_packet(int type,int apid=1){
  //build packet with given data (hex), type
  //ID(b3)|TM/TC(b1)|APID(h1)|length(h2)|type(h2)|data|CRC(h2)
  String packet="";
  packet+=toHex(myId,1);
  packet+="2";
  packet+=toHex(51*2,2);
  packet+=toHex(type,2);
  for (int i=0;i<50;i++){
  packet+=toHex(bufferH[i],2);
  }
  packet+="00000000";
  packet+="FF";//implement CRC
  packet="{\"type\":\"onPacket\",\"id\":\""+String(myId)+"\",\"data\":\""+packet+"\"}";
  USB.println(packet);
  return packet;
}

void ESATEPS::housekeeping(){
 //// Check and dispatch commands
 if(EPS.command!=0){handleCommand();}
  if (USB.available()){
  String cmd=USB.readStringUntil('\r');
    String identifier=cmd.substring(0,1);
    if(identifier=="@"){decode_tc_packet(cmd.substring(1,cmd.length()+1));}
  }
 ////
 //// EPS (Main) TM
 int channels[14]={A0,A1,A3,A4,A5,A6,A7,A2,A15,A8,A9,A13,A12,A14};
            // ->I12,V12,I5,V5,I3,V3,Iin,Vin,Ip2in,Vp2,Ip2out,ip1out,Vp1,Ip1in
 int x[14];
 int numAvgs[14]={5,1,5,1,5,1,5,1,5,1,5,1,5,5};
 
  for (int p=0;p<14;p++){
  double y=0;
  for (int i=0;i<numAvgs[p];i++){y+=double(analogRead(channels[p]))/numAvgs[p];delay(10);}
  x[p]=round(y);
  }

for(int i=2;i<14;i++){
  bufferH[2*i+1]=(x[i]&255);
  bufferH[2*i]=(x[i]>>8);
  }
//Software version
  bufferH[1]=(soft_v<<3);
//
///////// EPS (Bat) TM
  byte res[4];
  I2Cread(bqAddress,0x09,2,res);bufferH[28]=res[1];bufferH[29]=res[0]; //Vbatt
  I2Cread(bqAddress,0x3E,2,res);bufferH[30]=res[1];bufferH[31]=res[0]; //Vb1
  I2Cread(bqAddress,0x3F,2,res);bufferH[32]=res[1];bufferH[33]=res[0]; //Vb2
  I2Cread(bqAddress,0x0D,1,res);bufferH[48]=0;bufferH[49]=res[0];       //SOC
  I2Cread(bqAddress,0x08,2,res);bufferH[34]=res[1];bufferH[35]=res[0]; //Tbatt
  byte error=I2Cread(bqAddress,0x0a,2,res);bufferH[36]=res[1];bufferH[37]=res[0];//Ibatt
  bitWrite(EPSStatus,3,error==0); //EPSStatus.BAT
////////  Solar array TM
//
  error=I2Cread(0x4A,0x00,2,res);//TS1
  if (error==0){error=I2Cread(0x18,0x05,2,res);}
  if(error!=0){bufferH[38]=res[0];bufferH[39]=res[1];}
  else {bufferH[38]=0;bufferH[39]=0;}
  bitWrite(EPSStatus,2,error==0);
  error=I2Cread(0x49,0x00,2,res);//TS2
  if (error!=0){error=I2Cread(0x1C,0x05,2,res);}
  if(error==0){bufferH[40]=res[0];bufferH[41]=res[1];}
  else {bufferH[40]=0;bufferH[41]=0;}
  bitWrite(EPSStatus,1,error==0);
////////  DET TM 
  uint16_t channels_ADC;
  for (int i=0; i<3; i++){
  channels_ADC=readADC(i);
  bufferH[42+2*i+1]=(channels_ADC&255);
  bufferH[42+2*i]=(channels_ADC>>8);
  }
/////// EPS Status registers
bitWrite(EPSStatus,7,ENABLED5);bitWrite(EPSStatus,6,ENABLED3);bitWrite(EPSStatus,5,isOC2);bitWrite(EPSStatus,4,!digitalRead(OC1));
bufferH[3]=EPSStatus;
build_tm_packet(1,2);
}

String ESATEPS::toHex(int i,int L){
  String ch=String(i,HEX);
  while (ch.length()<L){ch="0"+ch;}
  return ch.substring(0,L);
}

void ESATEPS::updateMPPT(){
 if(!MPPTFIXED){
  double u1=0;
  if (MPPTMODE){
   u1=0;
   for (int i=0;i<2;i++){u1+=double(analogRead(A15))/2;}; 
   if(((u1>=previousI1)&&(MPPTDelta1>0))||((u1<previousI1)&&(MPPTDelta1<0))){MPPTDelta1=2;}
   else {MPPTDelta1=-2;}
    
  PWMValue1+=MPPTDelta1;
  if (PWMValue1>=max1){PWMValue1=max1;} 
  if (PWMValue1<=min1){PWMValue1=min1;}
    analogWrite(PWM1, PWMValue1);  
  delay(10);
  previousI1=u1; 
  }
  else {
  if (PWMValue1>=max1){MPPTDelta1=-1;}
  if (PWMValue1<=min1){MPPTDelta1=1;}
  PWMValue1+=MPPTDelta1;
  if (PWMValue1>=max1){PWMValue1=max1;} 
  if (PWMValue1<=min1){PWMValue1=min1;}
  analogWrite(PWM1, PWMValue1);
  }

  if (MPPTMODE){
   u1=0;
   for (int i=0;i<2;i++){u1+=double(analogRead(A14))/2;}; 
   if(((u1>=previousI2)&&(MPPTDelta2>0))||((u1<previousI2)&&(MPPTDelta2<0))){MPPTDelta2=2;}
   else {MPPTDelta2=-2;}
    
  PWMValue2+=MPPTDelta2;
  if (PWMValue2>=max1){PWMValue2=max1;} 
  if (PWMValue2<=min1){PWMValue2=min1;}
    analogWrite(PWM2, PWMValue2);  
  delay(10);
  previousI2=u1; 
  }
  else {
  if (PWMValue2>=max1){MPPTDelta2=-1;}
  if (PWMValue2<=min1){MPPTDelta2=1;}
  PWMValue2+=MPPTDelta2;
  if (PWMValue2>=max1){PWMValue2=max1;} 
  if (PWMValue2<=min1){PWMValue2=min1;}
  analogWrite(PWM2, PWMValue2);
  }
 }
}

uint16_t ESATEPS::readADC(int channel) {
  int I2C_address = 0x48;
  uint16_t res;
  uint16_t config=0x0003|0x0080|0x0100; 
  config |=0x0200;
  switch (channel)
  {
    case (0): config |= 0x4000;break;
    case (1): config |= 0x5000;break;
    case (2): config |= 0x7000;break;
	default: return 0;break;
  }
  // Set 'start single-conversion' bit
  config |=0x8000 ;//ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  Wire1.beginTransmission(I2C_address);
  Wire1.write((uint8_t)0x01);
  Wire1.write((uint8_t)(config>>8));
  Wire1.write((uint8_t)(config & 0xFF));
  Wire1.endTransmission();
  // Wait for the conversion to complete
  delay(1);
  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  byte result[2];
  byte error=I2Cread(I2C_address,0x00,2,result);
  bitWrite(EPSStatus,0,error==0);//if (error==0){EPSStatus |=0x01;}
  if (error==0){res=((result[0] << 8) | result[1])>>4;
  return (int16_t)res;}
  else return 0;
}


 void ESATEPS::switch_fun()
{
  detachInterrupt(OC2);
  isOC2=true;
  attachInterrupt(OC2,switch_fun_n,RISING);
}
void ESATEPS::switch_fun_n()
{
  //Each rotation, this interrupt function is run twice
  detachInterrupt(OC2);
  isOC2=false;
  attachInterrupt(OC2,switch_fun,FALLING);
}
void ESATEPS::decode_tc_packet(String hexstring){
 EPS.command=(int)strtol(hexstring.substring(4,6).c_str(),0,16);
 int length=(int)strtol(hexstring.substring(2,4).c_str(),0,16); 
 EPS.param=hexstring.substring(6,6+length).toInt();
 handleCommand();
}

void ESATEPS::invokeBSL(){
    SFRIE1 &= ~OFIE; /* Disable oscillator fault enable interrupt */
    //USB.end();
    delay(500);
    SYSBSLC &= ~(SYSBSLPE | SYSBSLOFF);
    __disable_interrupt(); /* Ensure no application interrupts occur while in BSL */
    /*
     * This sends execution to the BSL. When execution returns
     * to the user app, it will be via the reset vector, meaning
     * execution will re-start.
     */
     ((void (*)())0x1000)();
}

ESATEPS EPS;
