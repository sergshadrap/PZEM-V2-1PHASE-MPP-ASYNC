#include <Arduino.h>
#include <ESP.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <MppServer.h>
#include <MppDevice.h>
#include <MppHTTPCLient.h>
#include <stdio.h>
#include <IPAddress.h>

#define PZEM_VOLTAGE (uint8_t)0xB0
#define RESP_VOLTAGE (uint8_t)0xA0

#define PZEM_CURRENT (uint8_t)0xB1
#define RESP_CURRENT (uint8_t)0xA1

#define PZEM_POWER   (uint8_t)0xB2
#define RESP_POWER   (uint8_t)0xA2

#define PZEM_ENERGY  (uint8_t)0xB3
#define RESP_ENERGY  (uint8_t)0xA3

#define PZEM_SET_ADDRESS (uint8_t)0xB4
#define RESP_SET_ADDRESS (uint8_t)0xA4

#define PZEM_POWER_ALARM (uint8_t)0xB5
#define RESP_POWER_ALARM (uint8_t)0xA5

//#define RESPONSE_SIZE sizeof(PZEMCommand)
#define RESPONSE_SIZE 7
// #define RESPONSE_DATA_SIZE RESPONSE_SIZE - 2
#define RESPONSE_DATA_SIZE 5
#define PZEM_BAUD_RATE 9600
#define PZEM_ERROR_VALUE -1.0


const char *DeviceVersion = "MppPZEM 3.2.1";
static const char *P_PERIOD = "Period"; // Period - the frequency to read the analog input in sconds
static const char *RX_PIN = "RX_PIN";
static const char *TX_PIN = "TX_PIN";
static const char *READ_S = "READ_S";
static const char *COMMAND_S = "COMMAND_S";

static const char *properties[] = { //
        P_PERIOD, //   how often reporting to AM server, ms
        RX_PIN, // RX pin GPIO
        TX_PIN, // TX  pin GPIO
        READ_S, // delay between reading from PZEM 
        COMMAND_S,//delay in sending commands to PZEM
        NULL };
MppServer mppServer(DeviceVersion, properties);
class MppDevice volt;
class MppDevice curr;
class MppDevice powr;
class MppDevice ener;

IPAddress ip1(192,168,1,1);

uint8_t buffer[RESPONSE_SIZE];
unsigned long next = millis();

unsigned long last_command=millis();
 unsigned long last_reading=millis();
  unsigned long first_command=millis();
  
// PZEM004T pzem(12, 14);  //D5,D6 for Wemos D1

class PZEM004T    {

public:
PZEM004T(uint8_t receivePin, uint8_t transmitPin)
{
    SoftwareSerial *port = new SoftwareSerial(receivePin, transmitPin);
    port->begin(PZEM_BAUD_RATE);
    this->serial = port;
    this->_isSoft = true;
    Serial.printf("Serial initialized at:%d RX and %d TX\n",receivePin, transmitPin);
}
~PZEM004T()
{
    if(_isSoft)
        delete this->serial;
}

// private:
struct PZEMCommand {
    uint8_t command;
    uint8_t addr[4];
    uint8_t data;
    uint8_t crc;
};

    Stream *serial;

    bool _isSoft;
 //   unsigned long _readTimeOut = PZEM_DEFAULT_READ_TIMEOUT;

boolean send(const IPAddress &addr, uint8_t cmd, uint8_t data)
{
  
    PZEMCommand pzemcom;
//      first_command=millis();
//   serial->flush();
 while (serial->available())  serial->read(); // clear buffer 
// Serial.printf("Time for while action:%u ms Time:%u ms for command:%01X\n",first_command,millis(),cmd); 
// while (serial->available())  serial->read(); // clear buffer 
    pzemcom.command = cmd;
    for(int i=0; i<sizeof(pzemcom.addr); i++) 
        pzemcom.addr[i] = addr[i];
    pzemcom.data = data;

    uint8_t *bytes = (uint8_t*)&pzemcom;
    pzemcom.crc = crc(bytes, sizeof(pzemcom) - 1);

  int b= serial->write(bytes, sizeof(pzemcom));
 //   last_command=millis();
 //   Serial.printf("\nBuffer to send: command :%01X, address:%01X %01X %01X %01X , data:%01X ,crc:%01X Time:%u CRC:%01X,  Total send:%d\n",pzemcom.command,pzemcom.addr[0],pzemcom.addr[1],pzemcom.addr[2],pzemcom.addr[3],pzemcom.data,pzemcom.crc,millis(),pzemcom.crc,b);
serial->flush();
 return true;
}

uint8_t receive( void )
{
  if (serial->available()==0) {
//   Serial.printf("Serial low reading:%d bytes RESPONSE_SIZE:%d\n",serial->available(),RESPONSE_SIZE); 
    serial->flush();
    return 0xFF; }// false if nothing to read and no command in device
  
    buffer[0]=0; buffer[RESPONSE_SIZE-1]=0;

#ifdef PZEM004_SOFTSERIAL    
    if(_isSoft)
        ((SoftwareSerial *)serial)->listen();
#endif

 uint8_t len = 0;
 while (serial->available()   ||  len<(RESPONSE_SIZE))  {
   uint8_t c =(uint8_t)serial->read();
    buffer[len] = c; 
//   Serial.printf("Serial byte data :%01X len:%d Total byte available:%d\n",c,len,serial->available());
  if((buffer[0]!= RESP_VOLTAGE)&& (buffer[0]!=RESP_CURRENT) && (buffer[0]!=RESP_POWER) && (buffer[0]!= RESP_ENERGY)) {
// Serial.printf(" BAD Buffer[0]!: %01X\n",buffer[0]);
  serial->flush();   // clear buffer  
       return 0xFE;
  }
  len++;
}

 if(len<RESPONSE_SIZE) {
 //    Serial.printf("Short data! len:%d buffer: %01X %01X %01X %01X %01X %01X %01X available size:%d\n",len,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],serial->available());
 //       while (serial->available())  serial->read(); // clear buffer 
 serial->flush();
        return 0xFE;
 }
  if(buffer[RESPONSE_SIZE-1] != crc(buffer, len - 1)) {
  //         Serial.printf("Bad CRC! recived:%01X calculated:%01X data length:%d  Buffer: %01X %01X %01X %01X %01X %01X %01X\n",buffer[6],crc(buffer, len - 1),len-1,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);
 //           while (serial->available())  serial->read(); // clear buffer 
 serial->flush();
            return 0xFD;
          }
// Serial.printf("Response:%01X %01X %01X %01X %01X %01X %01X\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);

  //   serial->flush();     
    return buffer[0];
}

uint8_t crc(uint8_t *data, uint8_t sz)
{
    uint16_t crc = 0;
    for(uint8_t i=0; i<sz; i++)
        crc += *data++;
    return (uint8_t)(crc & 0xFF);
}
} *pzem;


void setup() {
  Serial.begin(115200);
  mppServer.setPropertyDefault(P_PERIOD, "1000");
  mppServer.setPropertyDefault(RX_PIN, "12");
  mppServer.setPropertyDefault(TX_PIN, "14");
  mppServer.setPropertyDefault(READ_S, "250");
  mppServer.setPropertyDefault(COMMAND_S, "500");
  
  pzem = new class PZEM004T(mppServer.getUnsignedProperty(RX_PIN),mppServer.getUnsignedProperty(TX_PIN));
 
  mppServer.manageDevice(&volt, getDefaultUDN(MppAnalog)+"V");
  mppServer.manageDevice(&curr, getDefaultUDN(MppAnalog)+"I");
  mppServer.manageDevice(&powr, getDefaultUDN(MppAnalog)+"P");
  mppServer.manageDevice(&ener, getDefaultUDN(MppAnalog)+"E");

  Serial.printf("\nMppServer booting: %s, mode=%d, version=%d\n",
      ESP.getResetReason().c_str(), ESP.getBootMode(),
      ESP.getBootVersion());


      mppServer.begin();

}

 int Ncycle=0;
 float p2,v1,i1,p1,e1=0; // declare it globally 
 
void loop() {

 //  bool notify;
  unsigned long now= millis();

  mppServer.handleClients(); // let the server handle any incoming requests
 mppServer.handleCommand(); // optional, handle user Serial input 

if(millis()-last_command >mppServer.getUnsignedProperty(COMMAND_S)) {
     switch (Ncycle) {
      case 1:
pzem->send(ip1, PZEM_VOLTAGE,0) ;
 break;
      case 2:
 pzem->send(ip1, PZEM_CURRENT,0);  
   break;
      case 3:
  pzem->send(ip1, PZEM_POWER,0);  
   break;
      case 4:
 pzem->send(ip1, PZEM_ENERGY,0);      
    break;
default:
    Ncycle =0;
     }       
  Ncycle++;   
  last_command=millis();
 ///Serial.printf("Flags  V:%d  A:%d  P:%d E:%d Ncycle:%d\n",flag_volt, flag_cur, flag_pow, flag_ener,Ncycle);
  }


if(millis()-last_reading >mppServer.getUnsignedProperty(READ_S)) {
  switch ( pzem->receive()) {
      case RESP_VOLTAGE:
    v1 = (buffer[1] << 8) + buffer[2] + (buffer[3] / 10.0);
     break;
     case RESP_CURRENT:
    i1=   (buffer[1] << 8) +  buffer[2] + (buffer[3] / 100.0);
     break;
      case RESP_POWER:
   p1= (buffer[1] << 8) + buffer[2];
//   if(p2!=p1) { Serial.printf("Power:%.2f W\n",p2); p1= p2;}
    break;
      case RESP_ENERGY:
     e1= ((uint32_t)buffer[1] << 16) + ((uint16_t)buffer[2] << 8) + buffer[3]; 
    break;
    case 0xFF:  // No data in serial
    break;
  default:
//      Serial.printf("Error reading serial! Code:%d\n",r);
//     Serial.printf("Response :%01X,%01X,%01X,%01X\n",buffer[0],buffer[1],buffer[2],buffer[3]);
      break;
  }
   last_reading=millis();
}
 
  if (now > next) {
//  Serial.printf(" Voltage :%.2fV  , %01X,%01X,%01X,%01X\n",v1,buffer[0],buffer[1],buffer[2],buffer[3]);
  //  if( v1!=PZEM_ERROR_VALUE && i1!=PZEM_ERROR_VALUE ){
        Serial.printf(" Voltage :%.2fV  Current:%.2fA Power:%.2fW Energy:%.2fWt \n",v1,i1,p1,e1); 
        volt.update(VALUE,String(v1)); volt.update(STATE,"on"); 
        curr.update(VALUE,String(i1)); curr.update(STATE,"on");
        powr.update(VALUE,String(p1)); powr.update(STATE,"on");
        ener.update(VALUE,String(e1)); ener.update(STATE,"on");
/*    } else {
        Serial.println("Error reading F1");
        notify=volt.put(STATE,"off");
         notify=curr.put(STATE,"off");
          notify=powr.put(STATE,"off");
           notify=ener.put(STATE,"off");
    }*/

Serial.printf("heap=%d at %lus \n", ESP.getFreeHeap(), now / 1000);
 next = now + mppServer.getUnsignedProperty(P_PERIOD);
  }
}
