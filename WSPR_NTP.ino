/*
 * Very simple WSPR beacon using NTP for time synchronisation and an Si5351 oscillator.
 * Created on a WeMos D1 R2 (ESP8266 on Arduino style board).
 * Peter Marks VK3TPM
 *
 * Heavily based on work by Jason Milgram & Michael Margolis
 */

/*

  Udp NTP Client

  Get the time from a Network Time Protocol (NTP) time server
  Demonstrates use of UDP sendPacket and ReceivePacket
  For more on NTP time servers and the messages needed to communicate with them,
  see http://en.wikipedia.org/wiki/Network_Time_Protocol

  created 4 Sep 2010
  by Michael Margolis
  modified 9 Apr 2012
  by Tom Igoe
  updated for the ESP8266 12 Apr 2015
  by Ivan Grokhotkov

  This code is in the public domain.

*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <si5351.h>
#include <JTEncode.h>
#include "Wire.h"

#ifndef STASSID
#define STASSID "MONWIFI"   // CHANGE WIFI SSID  !
#define STAPSK  "MDPWIFI"   // CHANGE WIFI PASSWORD  !
#endif

// Define pins fir filter relay
#define B_40M D5
#define B_10_15M D3
#define B_17_20M D0


const char * ssid = STASSID; // your network SSID (name)
const char * pass = STAPSK;  // your network password

unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

#define TONE_SPACING            146           // ~1.46 Hz
#define WSPR_CTC                10672         // CTC value for WSPR
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT
#define CORRECTION              99000             // Change this for your ref osc
#define SYMB_LENGTH             682           // lenth of a symbol in ms
#define FREQ_40M                7038600       // 40 meter band center frequency
#define FREQ_20M                14095600      // 20 meter band center frequency
#define FREQ_15M                21094600      // 15 meter band center frequency
#define FREQ_10M                28124600      // 10 meter band center frequency
#define FREQ_2M                 144489000     // 2 meter band center frequency


Si5351 si5351;
JTEncode jtencode;
//unsigned long freq_WSPR [5] = {7038600, 14095600, 21094600, 28124600, 144489000};



char call[7] = "MYCALL";                        // Change this
char loc[5] = "JN38";                           // Change this


unsigned long freq_WSPR_40M = 7038600;
uint8_t dbm_40M = 3;
unsigned long freq_WSPR_20M = 14095600;
uint8_t dbm_20M = 10;
unsigned long freq_WSPR_15M = 21094600;
uint8_t dbm_15M = 10;
unsigned long freq_WSPR_10M = 28124600;
uint8_t dbm_10M = 10;
unsigned long freq_WSPR_6M = 50293000;
uint8_t dbm_6M = 10;
unsigned long Offset = 1567;    // Change this


unsigned long freq = freq_WSPR_40M + Offset;                
uint8_t dbm = dbm_40M;
unsigned int counter = 4;
uint8_t tx_buffer[SYMBOL_COUNT];

void setupSi5351() {
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, CORRECTION);
  if(i2c_found == true) {
    Serial.println("i2c found!");
    // Set CLK0 output
    si5351.set_freq(freq * 100ULL, SI5351_CLK1);
    si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); // Set output power : 2mA = 2dBm,  8mA = 10 dBm -   Param is : SI5351_DRIVE_2MA SI5351_DRIVE_8MA   
    si5351.set_clock_pwr(SI5351_CLK1, 0); // Disable the clock initially
  } else {
    Serial.println("i2c NOT found!");
    while(1) {};
  }
}

// Loop through the string, transmitting one character at a time.
void transmitWSPR()
{
    uint8_t i;

    jtencode.wspr_encode(call, loc, dbm, tx_buffer);
    Serial.println(freq);
    Serial.println("Start TX");  
    si5351.set_freq(freq * 100ULL, SI5351_CLK1);
    
    // Reset the tone to 0 and turn on the output
    si5351.set_clock_pwr(SI5351_CLK1, 1);
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Now do the rest of the message
    for(i = 0; i < SYMBOL_COUNT; i++)
    {
        Serial.print("|");
        si5351.set_freq((freq * 100ULL) + (tx_buffer[i] * TONE_SPACING), SI5351_CLK1);
        //        si5351.set_freq((freq * 100ULL) , SI5351_CLK1);
        delay(SYMB_LENGTH);  //  Wait for next symbol
    }
    
    
    Serial.println();
    // Turn off the output
    si5351.set_clock_pwr(SI5351_CLK1, 0);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Stop TX");  
}

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

void setup() {
  pinMode(B_40M, OUTPUT);
  pinMode(B_10_15M, OUTPUT);
  pinMode(B_17_20M, OUTPUT);
  digitalWrite(B_40M,LOW);
  digitalWrite(B_10_15M,LOW);
  digitalWrite(B_17_20M,LOW);

    
   
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("Setting up Si5351");
  setupSi5351();
  
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
}

void loop() {
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  } else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    int minute = (epoch  % 3600) / 60;
    Serial.print(minute); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    int second = epoch % 60;
    if (second < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(second); // print the second

    // calculate time to wait before next 2 minute slot
    int minutesToWait = ((minute + 1) % 2);
    Serial.print("minutesToWait = ");
    Serial.println(minutesToWait);
    int secondsToWait = (minutesToWait * 60) + (60 - second);
    Serial.print("secondsToWait = ");
    Serial.println(secondsToWait);
    delay(secondsToWait * 1000);


   if (counter == 0) {
      freq = freq_WSPR_40M + Offset;  
      dbm = dbm_40M;
      digitalWrite(B_40M,HIGH);
    };
    
    if (counter == 1) {
      freq = freq_WSPR_20M + Offset;  
      dbm = dbm_20M;
      digitalWrite(B_17_20M,HIGH);
    };
    if (counter == 2) {
      freq = freq_WSPR_15M + Offset;  
      dbm = dbm_15M;
      digitalWrite(B_10_15M,HIGH);
    };

    if (counter == 3) {
      freq = freq_WSPR_10M + Offset;  
      dbm = dbm_10M;
      digitalWrite(B_10_15M,HIGH);
    };
    Serial.println("new freq before TX : ");
    Serial.println(freq);

    if (counter == 4) {
      freq = freq_WSPR_6M + Offset;  
      dbm = dbm_6M;
      //digitalWrite(B_10_15M,HIGH);
    };

    
    Serial.println("new freq before TX : ");
    Serial.println(freq);
    
    Serial.println("WSPR TX start");

  
    transmitWSPR();
    digitalWrite(B_40M,LOW);
    digitalWrite(B_10_15M,LOW);
    digitalWrite(B_17_20M,LOW);
    Serial.println("WSPR TX ends");

    
  }
  // wait ten seconds before asking for the time again
  delay(10000);
  counter = (counter+1)%5;
  Serial.print("Freq-index = ");
  Serial.println(counter);
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
