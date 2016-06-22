#include <Time.h>
#include <Wire.h>
#include <AltSoftSerial.h>
AltSoftSerial altSerial;
//#include <SoftwareSerial.h>
//SoftwareSerial altSerial(8, 9); // RX, TX
#define RTC_ADD 0x68
#define BIT_PERIOD 840 // us
#define BUFF_SIZE 64
volatile long data[BUFF_SIZE];
volatile uint8_t in;
volatile uint8_t out;
volatile unsigned long last_us;
volatile int int1_flag;
tmElements_t tm;

//PROGMEM answer strings from ESP
char str_buffer[64];
const char STR_OK[] PROGMEM =     "OK";
const char STR_BOOT[] PROGMEM =   "GOT IP";  //End of bootup message
const char STR_CIPSEND[] PROGMEM = "+CWJAP:\"";
const char STR_SENDMODE[] PROGMEM = ">";

 
//--------------------------------------------
// Interrupt Services Rutines
//--------------------------------------------
ISR(INT0_vect) {
   unsigned long us = micros();
   unsigned long diff = us - last_us;
   if (diff > 20 ) {
      last_us = us;
      int next = in + 1;
      if (next >= BUFF_SIZE) next = 0;
      data[in] = diff;
      in = next;
   }
}
    
ISR(INT1_vect) { //interrupts at pin 3
    int1_flag = true;    
}

//--------------------------------------------
// Arduino Setup and loop
//--------------------------------------------
void setup() {
//----Init Arduino-----
  in = out = 0;
  last_us = micros();
  EIMSK |= 0b00000011; //mask for INT1 and INT0  
//  delay(1000);
  EICRA |= 0b00001011; //Falling INT1 | RISING INT0 
  
  Wire.begin();
  altSerial.begin(9600);
  Serial.begin(19200);

  Serial.print(F("MR Setup\r\n"));
  resetESP();

  pinMode(2, INPUT);      
  digitalWrite(2, HIGH); 
  pinMode(3, INPUT);      
  digitalWrite(3, HIGH); 

//TODO check connection, connet if not

  getNTP();
  Serial.println(); 
  Serial.print(F("NTP time: "));
  printDate(Serial);
  Serial.print(" ");
  printTime(Serial);
  Serial.println(); 
  //todo set clock if incorrect only
//  setRTC();
  
  enableRTCbits(0x0e,0b00000100); //INTCN
  enableRTCbits(0x0e,0b00000110); //INTCN arm al2
  
  getRTC();
 
  setRTC5minAlrm(tm.Minute+5<60?tm.Minute+5:tm.Minute+5-60);
  
  Serial.print(F("RTC time: "));
  printDate(Serial);
  Serial.print(" ");
  printTime(Serial);
  Serial.print(" ");  
  Serial.print(F(" TEMP: "));
  Serial.println(getTemperature());

}

volatile unsigned int statusFlag;
volatile float imports;
volatile float exports;

void loop() {
  while (altSerial.available()) {
    Serial.print((char)altSerial.read());
  }
  while (Serial.available()) {
    altSerial.print((char)Serial.read());
  }
  if(int1_flag){
    Serial.print(F("External Interrupt detected: "));
    Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231 
    Wire.write(0x0F);               //Tell the device which address we want to read or write
    Wire.write(0b00000000);         //Write the byte.  The last 0 bit resets Alarm 1
    Wire.endTransmission();
    //TODO:saveDataToPVoutput();
   getRTC();
   printDate(Serial);        Serial.print(" ");
   printTime(Serial);        Serial.print(" ");
   Serial.print(imports);    Serial.print(" ");
   Serial.print(exports);    Serial.print(" ");
   Serial.print(statusFlag); Serial.println(); 
   setRTC5minAlrm(tm.Minute+5<60?tm.Minute+5:tm.Minute+5-60);
   int1_flag=false;           //Finally clear the flag we use to indicate the trigger occurred
  }  
  
//  int rd = 0;
  int rd = decode_buff();
  if (!rd) return;
  if (rd==3) {
    rd=4;
   Serial.print(F(">>>>>>>>>>>>>>>>>>>>>>"));
   Serial.print(imports);    Serial.print(" ");
   Serial.print(exports);    Serial.print(" ");
   Serial.print(statusFlag); Serial.println();  
   getRTC();  
   if (saveData()) {
    altSerial.print(F("AT+CIPCLOSE=4\r\n"));
    waitForString(getString(STR_OK), 2, 5000);
    //delay(5000);
    if (saveData()) {
      //TODO hard reset esp try again pin7
      resetESP();
      if (!saveData()) dbg((" Sx "));
    } else {
      dbg((" S2 "));
    }
   } else {
    dbg((" S1 "));
   }
//   getReply( 1500 ,true);
  }
}

//--------------------------------------------
// ESP8266 / Network functions
//--------------------------------------------

void resetESP() {
  pinMode(7, OUTPUT);  
  digitalWrite(7, LOW);
  delay(100);
  digitalWrite(7, HIGH); 

   
  waitForString(getString(STR_BOOT), 6, 120000);
  
  //todo return true if worked try again if not?  

  altSerial.print(F("AT+CWMODE=1\r\n"));
  waitForString(getString(STR_OK), 2, 1500);
  
  altSerial.print(F("AT+CIPMUX=1\r\n"));
  waitForString(getString(STR_OK), 2, 2500);
  
  altSerial.print(F("AT+CIPSERVER=1,8888\r\n"));
  waitForString(getString(STR_OK), 2, 2500);

}

void getNTP() {
  altSerial.print(F("AT+CIPSTART=4,\"UDP\",\"0.pool.ntp.org\",123\r\n"));
  waitForString(getString(STR_OK), 2, 1000);
  altSerial.print(F("AT+CIPSEND=4,48\r\n"));    
  waitForString(getString(STR_OK), 2, 1000);

  byte packetBuffer[48];
  memset(packetBuffer, 0, 48); 
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  altSerial.write(packetBuffer,48);
  waitForString(getString(STR_OK), 2, 1000);

  char* lookFor = "+IPD,4,48:";
  uint8_t index = 0;
  uint8_t tempPos = 0;
  uint8_t found = 0;
  long int time = millis();
  while((time + 2500) > millis()) {
    while(altSerial.available()) {
      char c = altSerial.read();
      Serial.print(c); //Serial.print("=>"); Serial.println(lookFor[index]);
      index = (c==lookFor[index]) ? index+1 :0;
      if (found && tempPos+1<=48) packetBuffer[tempPos++] = c;
      if (index==strlen(lookFor)) found = 1;       
    }
  } 

  unsigned long secsSince1900;
  // convert four bytes starting at location 40 to a long integer
  secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
  secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
  secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
  secsSince1900 |= (unsigned long)packetBuffer[43];
  altSerial.print(F("AT+CIPCLOSE=4\r\n"));
  waitForString(getString(STR_OK), 2, 1500);
  Serial.print("Seconds since Jan 1 1900 = " );Serial.println(secsSince1900);
  breakTime(secsSince1900 - 2208988800UL + 10 * SECS_PER_HOUR, tm);
  tm.Year -=30;
}

uint8_t saveData() {

  altSerial.print(F("AT+CIPSTART=4,\"TCP\",\"192.168.178.51\",8080\r\n"));

  //Check for connection errors
  if (!waitForString(getString(STR_OK), 2, 3000)) {
    Serial.println(F(">>>>>>>>>>>>>>>>>>>Error CIPSTART to server"));
    return 1;
  }
  
  altSerial.print(F("AT+CIPSENDEX=4,122\r\n"));
  if (!waitForString(getString(STR_OK), 2, 3000)) {
    Serial.println(F(">>>>>>>>>>>>>>>>>>>Error CIPSEND to server"));
    return 1;
  }
  
  altSerial.print(F("GET /SaveReading.do?a="));
  printDate(altSerial);
  altSerial.print(F("&b="));
  printTime(altSerial);
  altSerial.print(F("&c="));
  altSerial.print(imports);
  altSerial.print(F("&d="));
  altSerial.print(exports);
  altSerial.print(F("&e="));
  altSerial.print(statusFlag);
  altSerial.print(F("&f="));  
  altSerial.print(getTemperature());
  altSerial.print(F(" HTTP/1.1\r\nHost: 192.168.178.51\r\n\r\n\r\n"));
  altSerial.print("\\");
  altSerial.print('0');
  //getReply(2500,true);
  if (!waitForString(getString(STR_OK), 2, 3000)) {
    Serial.println(F(">>>>>>>>>>>>>>>>>>>Error URL to server"));
    return 1;
  }
  altSerial.print(F("AT+CIPCLOSE=4\r\n"));
  //getReply( 2000,true );
  waitForString(getString(STR_OK), 2, 2000);
  return 0;
}

void dbg(char* str) {
//  Serial.print(str);
  altSerial.print(F("AT+CIPSENDEX=0,128\r\n"));
  waitForString(getString(STR_OK), 2, 1000);
  altSerial.print(str);
  altSerial.print("\\0");
}

//void getReply(int wait,bool printReply) {//, String lookFor, ){
//  int tempPos = 0;
//  long int time = millis();
//  while((time + wait) > millis()) {
//    while(altSerial.available()) {
//      char c = altSerial.read();
//      if (printReply) Serial.print(c);
//    }
//  } 
//}

bool waitForString(char* input, uint8_t length, unsigned int timeout) {
  unsigned long end_time = millis() + timeout;
  int current_byte = 0;
  uint8_t index = 0;
  while (end_time >= millis()) {
    while(altSerial.available()) {
      char c = altSerial.read();
      Serial.print(c);
      current_byte = c;
      if (current_byte != -1) {
        //Search one character at a time
        if (current_byte == input[index]) {
          index++;
          //Found the string
          if (index == length) return true;
          //Restart position of character to look for
        } else {
          index = 0;
        }
      }
    }
  }  
  //Timed out
  return false;
}

//Return char string from PROGMEN
char* getString(const char* str) {
  strcpy_P(str_buffer, (char*)str);
  return str_buffer;
}

//--------------------------------------------
// RTC DS3231 real time clock functions
//--------------------------------------------

void setRTC(){ //byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year
  // sets time and date data to DS3231
  Wire.beginTransmission(RTC_ADD);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(tm.Second)); // set seconds
  Wire.write(decToBcd(tm.Minute)); // set minutes
  Wire.write(decToBcd(tm.Hour)); // set hours
  Wire.write(decToBcd(tm.Wday)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(tm.Day)); // set date (1 to 31)
  Wire.write(decToBcd(tm.Month)); // set month
  Wire.write(decToBcd(tm.Year)); // set year (0 to 99)
  Wire.endTransmission();
}

//void readRTC(byte *second,byte *minute,byte *hour,byte *dayOfWeek,byte *dayOfMonth,byte *month,byte *year){
//  Wire.beginTransmission(RTC_ADD);
//  Wire.write(0x00); // set DS3231 register pointer to 00h
//  Wire.endTransmission();
//  Wire.requestFrom(RTC_ADD, 7);
//  // request seven bytes of data from DS3231 starting from register 00h
//  *second = bcdToDec(Wire.read() & 0x7f);
//  *minute = bcdToDec(Wire.read());
//  *hour = bcdToDec(Wire.read() & 0x3f);
//  *dayOfWeek = bcdToDec(Wire.read());
//  *dayOfMonth = bcdToDec(Wire.read());
//  *month = bcdToDec(Wire.read());
//  *year = bcdToDec(Wire.read());
//}

void getRTC() {
  //TimeElements tm;
  Wire.beginTransmission(RTC_ADD);
  Wire.write(0x00); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADD, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  tm.Second = bcdToDec(Wire.read() & 0x7f); 
  tm.Minute = bcdToDec(Wire.read());; 
  tm.Hour = bcdToDec(Wire.read() & 0x3f); 
  tm.Wday = bcdToDec(Wire.read());   // day of week, sunday is day 1
  tm.Day  = bcdToDec(Wire.read());
  tm.Month = bcdToDec(Wire.read()); 
  tm.Year = bcdToDec(Wire.read());
  //return makeTime(tm);
}

float getTemperature() {
    byte temp;
    Wire.beginTransmission(RTC_ADD);
    Wire.write(0x11);
    Wire.endTransmission();
    Wire.requestFrom(RTC_ADD, 2);
    temp = Wire.read();  // Here's the MSB
    return float(temp) + 0.25*(Wire.read()>>6);
}

void enableRTCbits(byte address, byte mask) {
  Wire.beginTransmission(RTC_ADD);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADD, 1);
  byte temp_buffer = Wire.read() | mask;
  Wire.beginTransmission(RTC_ADD);
  Wire.write(address);
  Wire.write(temp_buffer); 
  Wire.endTransmission();
}

void setRTC5minAlrm(uint8_t mins) {
  Wire.beginTransmission(RTC_ADD);
  Wire.write(0x0B);
  Wire.write(decToBcd(mins) & 0b01111111);
  Wire.write(0b10000000); //hour
  Wire.write(0b10000000); 
  Wire.endTransmission();
}

void printDate(Stream& serial){
  serial.print(F("20"));
  serial.print(tm.Year, DEC);
  serial.print(F("-"));
  if (tm.Month<10) serial.print("0");
  serial.print(tm.Month, DEC);
  serial.print(F("-"));
  if (tm.Day<10) serial.print("0");
  serial.print(tm.Day, DEC);
}

void printTime(Stream& serial){
  if (tm.Hour<10) serial.print("0");
  serial.print(tm.Hour, DEC);
  serial.print(":");
  if (tm.Minute<10) serial.print("0");
  serial.print(tm.Minute, DEC);
  serial.print(":");
  if (tm.Second<10) serial.print("0");
  serial.print(tm.Second, DEC);

}

byte decToBcd(byte val){ return( (val/10*16) + (val%10) );}
byte bcdToDec(byte val){ return( (val/16*10) + (val%16) );}

//--------------------------------------------
// IRDx decoding
//--------------------------------------------
volatile float last_data;
volatile uint8_t sFlag;
volatile float imps;
volatile float exps;
uint16_t idx=0;
uint8_t byt_msg = 0;
uint8_t bit_left = 0;
uint8_t bit_shft = 0;
uint8_t pSum = 0;
uint16_t BCC = 0;
volatile uint8_t eom = 0;
  
static int decode_buff(void) { //decodig IRDx data
// Serial.print(F("MR decode buff\r\n"));
   if (in == out) return 0;
   int next = out + 1; if (next >= BUFF_SIZE) next = 0;
   //int next = (out + 1 >= BUFF_SIZE)? 0 : out + 1;
   int p = (((data[out]) + (BIT_PERIOD/2)) / BIT_PERIOD);
// Serial.print(p); Serial.print(F(" ")); //if (p>500) //Serial.print(F("{")); Serial.print(p); Serial.print(F("}"));
   if (p>500) { //period between IRDx messages
    Serial.print(F("\n\r\n\r#################################################<-\n\r"));
     idx = BCC =  imps = exps = sFlag = bit_left = bit_shft = pSum=0;
     out = next;
     dbg(("#"));
     return 0;
   }
   bit_left = (4 - (pSum % 5));
   bit_shft = (bit_left<p)? bit_left : p;
   pSum = (pSum==10)? p : ((pSum+p>10)? 10: pSum + p);
   if (eom==0b00011111 && pSum>=7) {
      pSum=pSum==7?11:10;
      //eom=0;   
   }

   if (bit_shft>0) {
      byt_msg >>= bit_shft;
      if (p==2) byt_msg += 0x40<<(p-bit_shft);
      if (p==3) byt_msg += 0x60<<(p-bit_shft);
      if (p==4) byt_msg += 0x70<<(p-bit_shft);   
      if (p>=5) byt_msg += 0xF0;
    }
//    Serial.print(p); Serial.print(" ");Serial.print(pSum);Serial.print(" ");    
//    Serial.print(bit_left);Serial.print(" ");Serial.print(bit_shft);Serial.print(" ");    
//    Serial.println(byt_msg,BIN);
    if (pSum >= 10) {
       idx++;
       if (idx==1 && byt_msg==0x01) eom |= 0b00000001;
       if (idx==2 && byt_msg==0x4F) eom |= 0b00000010;
       if (idx==3 && byt_msg==0x42) eom |= 0b00000100;
       if (idx==4 && byt_msg==0x02) eom |= 0b00001000;
       if (idx>4 && eom < 0b00001111) {;
        out = next;    
        return 0;
       }
       
       if (idx==327 && byt_msg == 0x03) eom |= 0b00010000;
       if (idx!=328) BCC=(BCC+byt_msg)&255;
       Serial.print("[");Serial.print(idx);Serial.print(":");Serial.print(byt_msg,HEX); Serial.print("]");
//      Serial.print(F("\n\r==================== "));
//      Serial.print(eom, BIN);
//      Serial.print(F(" ===================<-\n\r"));
       if (eom==0b00001111 || eom==0b00011111) {
  //       Serial.print(eom, BIN);
         if (idx>=95 && idx<=101)  
            imps += ((float)byt_msg-48) * pow(10 , (101 - idx));
         if (idx==103) 
            imps += ((float)byt_msg-48) / 10;
         if (idx>=114 && idx<=120) 
            exps += ((float)byt_msg-48) * pow(10 , (120-idx));
         if (idx==122) 
            exps += ((float)byt_msg-48) / 10;
         if (idx==210) 
            sFlag = (byt_msg-48)>>3; //1=Exporting ; 0=Importing
          
         if (idx==328) {
            Serial.println(F("")); Serial.print(F("---->>>>"));
            Serial.print(imps); Serial.print(F("\t"));
            Serial.print(exps); Serial.print(F("\t"));
            Serial.print(sFlag); Serial.print(F("\t |pSum: ")); 
            Serial.print(pSum); Serial.print(F("\tLastRead: "));              
            Serial.print(byt_msg,BIN); Serial.print(F("\tBCC: ")); //BCC
            Serial.print((byt_msg>>(pSum==10?(((~BCC)&0b1000000)?0:1):2)),BIN); Serial.print(F("==")); //BCC
            Serial.print((~BCC)&0x7F,BIN); Serial.print("<-calc\n\rX-X-X-X-X-X-X-X-X-X-X-X-X-X-X-X-X-X-X-X-X-X\n\r"); //BCC           
            eom=idx=0;
            if ((byt_msg>>(pSum==10?(((~BCC)&0b1000000)?0:1):2))==((~BCC)&0x7F)) {
               if (last_data != (imps + exps + sFlag)) {
                  imports=imps;
                  exports=exps;
                  statusFlag=sFlag;
                  last_data = imps + exps + sFlag;
                  out = next;
                  return 3;
               }
               dbg(("+"));
            }
         }  
       }
    }
    out = next;
    return 0;
}

