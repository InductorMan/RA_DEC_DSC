/*
  Based on:
    Very Low Cost Digital Setting Circles
    Copyright (c) 2017 Vladimir Atehortua. All rights reserved.
    This program is free software: you can redistribute it and/or modify
    it under the terms of the version 3 GNU General Public License as
    published by the Free Software Foundation.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with this program.
    If not, see <http://www.gnu.org/licenses/>

  This variant is:
    -A two encoder RA/DEC variant, no accelerometer. AMT112S encoders with index pulse,
    and push-pull outputs.
    -DNS captive portal routing to a webpage which serves a settable RA/DEC readout.
    -uses a revised encoder interrupt service routine strategy that seems not to ever miss
    pulses
    -provides optional debug encoder error reporting

  NOTE: debug encoder error reporting is NOT guaranteed to be correctly initialized to zero 
  errors. There seems to be some probability of single initial error per channel (maybe encoder state
  changes between initial state readout and ISR attachment?)

  This variant has ONLY been tested on ESP8266 board package version 2.5.0 and in Arduino
  1.8.4; when I tried to use the latest ESP8622 board package in Arduino 1.8.4, the example  
  sketches did not compile. 

  This variant is not "good" code. Several possible consistency checking issues exist (such as 
  unbounded growth of offset variables with repeated setting, and IIR overflow with very high encoder
  counts). When indexing enoders are used and the index pulses work correctly the numerical 
  consistency issues are moot. Other consistency check issues remain. The style is inconsistent from
  the hand of several subsequent authors, and no attempt has been made to achieve sylistic consistency.

  -InductorMan
*/

/**
    Hardware used:
    Adafruit Huzzah ESP8266 Feather 

    Dec encoder on pins 14, 12, and index on 13. RA encoder on 2 and 15, and index on 0.
    
    External gating circuit needed to allow proper boot pin states at startup (pin 16 
    active low output causes pins 0, 2, and 15 to be driven by the encoder: high on 
    pin 16 causes the pins to be undriven by the encoder and allows pullup/down 
    resistors to do their job during boot

    One could test the code without external gating circuitry by booting with the 
    pin (0 2 15) encoder unplugged and plugging in after boot.

    The gating circuits use are transistor based, but a 3.3V powered tristate buffer 
    should work.
    
    
*/

/*
put a "#define REVERT_TO_ORIGINAL_LIB" here if it's desired to use the original separate-
object-per-encoder, separate-interrupt-service-routine-per-encoder scheme that results from
the typical Stoffregen encoder library utilization. Otherwise we mash together both encoders'
logic into a single object so that there is only one shared ISR, which seems to fix the 
ESP8266 dropped edges/race condition/whatever causes encoder errors on ESP8266.
REVERT_TO_ORIGINAL_LIB is expected to cause encoder errors when both encoders slew rapidly
and simultaneously.

Note that it seems that sometimes the error counters register a single error on boot even 
with the revised encoder ISR in use. However no errors have been seen after the initial error
in limited testing.
*/
//#define REVERT_TO_ORIGINAL_LIB

//put a "#define USE_INDEX_PIN" here if the encoders are to have index pin inputs
#define USE_INDEX_PIN

//#define DEBUG

#define DEBUG_PERIOD  300    // how many milliseconds must have elapsed since last debug output in order to take a new one


//specify temporal filter constants for encoder state change smoothing
#define FILTER_EXP 4 // Filter will settle 2^-FILTER_EXP of the way to the final value on each update
#define FILTER_PERIOD 15 //Filter update period: filter time constant is FILTER_PERIOD / (2^-FILTER_EXP * 0.69)

//note that setting FILTER_PERIOD shorter than the main loop execution rate will of course not make the actual
//filter period faster. 
//note that it's easy to set FILTER_PERIOD too long or FILTER_EXP too large and create laggy, slow response
//note that no sanity checking is done on FILTER_EXP: overflows are possible with excessive value


//hard coded encoder mounting orentation correction constant
//there is no meaningful absolute encoder mounting orientation orientation correction if the encoders don't
//have an index, so we'll just set these to zero if there's no index input.
#ifdef USE_INDEX_PIN
//include any incidental filter step-count gain in the encoder mounting orentation correction constant
#define DEC_ENC_ZERO ( 2 << FILTER_EXP )
#define RA_ENC_ZERO ( 1 << FILTER_EXP )
#else
#define DEC_ENC_ZERO 0
#define RA_ENC_ZERO 0
#endif




// number of steps in a full circle, including any incidental filter gain
// (gain not guaranteed to contribute any actual resolution improvement)
#define STEPS_IN_FULL_CIRCLE  ( 16384 << FILTER_EXP )    


// Based loosely on "Encoder Library by Paul Stoffregen" with added index pulse input, 
//and now two encoders in one ISR to fix undiagnosed ESP8266 race condition of some sort
#include "src/TwoEncoder/TwoEncoder.h"  


#include <Wire.h>     // built in library from Arduino IDE
#include <ESP8266WiFi.h>  // built in library from Arduino IDE
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "webpage.h"

//angle unit vonversion factors
#define STEPS_TO_RA_SEC (86400.0f / (float)STEPS_IN_FULL_CIRCLE)
#define STEPS_TO_DEC_MIN (21600.0f / (float)STEPS_IN_FULL_CIRCLE)

#define DEC_MIN_IN_FULL_CIRCLE (360 * 60)
#define RA_SEC_IN_FULL_CIRCLE (24 * 60 * 60)

//Dec range to report (conventionally -90 to +90, but here we retain -180 to +180 to resolve
//mirror quadrant ambiguity)
#define DEC_MAX_SEC (DEC_MIN_IN_FULL_CIRCLE / 2)
#define DEC_MIN_SEC (DEC_MAX_SEC - DEC_MIN_IN_FULL_CIRCLE)

//time unit conversion factors
#define SEC_PER_SIDEREAL_DAY 86164.1f
#define SEC_PER_DAY 86400.0f



const byte DNS_PORT = 53;
IPAddress ip(1, 2, 3, 4);     // The "telescope IP address" that Skysafari should connect to is 1.2.3.4 which is easy to remember.
IPAddress gateway(1, 2, 3, 4);
IPAddress subnet(255, 255, 255, 0);

DNSServer dnsServer;
ESP8266WebServer webServer(80);

WiFiServer server(4030);    // 4030 is the default port Skysafari uses for WiFi connection to telescopes
WiFiClient remoteClient;    // represents the connection to the remote app (Skysafari)
#define WiFi_Access_Point_Name "InductorMan's DSC (no internet access)"   // Name of the WiFi access point this device will create for your tablet/phone to connect to.

//debug variables
long oldPosition  = -999;
long oldPosition2  = -999;

//manual circle re-setting offsets
long raOffset = 0;
long decOffset = 0;
long haOffset = 0;

//temporal filter state variables
int32_t filtRa;
int32_t filtDec;


//these pin assignments require gating circuitry outside of the ESP8266 in order to allow the pins to float during startup.
//pins 0, 2, and 15 are strapped to special states to define boot behavior and cannot be driven until booted. Gating circuit
//control is active low output of Pin 16 (which is also internally pulled up at startup, and so must be interpreted active low to 
//preclude inadvertent driving of pins before boot).

#ifndef REVERT_TO_ORIGINAL_LIB
#ifdef USE_INDEX_PIN
TwoEncoder raDecEnc(2, 15, 0, 14, 12, 13);
#else
TwoEncoder raDecEnc(2, 15, 14, 12);
#endif
#else
#ifdef USE_INDEX_PIN
TwoEncoder raEnc(2, 15, 0);
TwoEncoder decEnc(14, 12, 13);
#else
TwoEncoder raEnc(2, 15);
TwoEncoder decEnc(14, 12);
#endif
#endif



long startTime;


long last_debug = 0;  // millisecond timestamp of last printout, to print only every XXX milliseconds
long lastFilter = 0; // filter update timer. 

//captive portal webpage, to allow setting of circles
String s = webpage;

//get RA in right ascension seconds
//convert encoder position to right ascension seconds, by first converting to hour angle in seconds, and then subtracting
//elapsed time in sidereal seconds and an adjustable offset (for setting, since elapsed time will bear no relationship
//to actual local sidereal time)
long getRaSec ()
{
  long polAng = filtRa - RA_ENC_ZERO;

  long ra = (long)(  ((float)polAng * STEPS_TO_RA_SEC) - (float)((millis()-startTime)/1000)*(SEC_PER_DAY / SEC_PER_SIDEREAL_DAY) - raOffset  );

  //force angle positive. 
  while(ra < 0){
    ra += RA_SEC_IN_FULL_CIRCLE;
  }

  //fix angles >= 24h 
  while(ra >= RA_SEC_IN_FULL_CIRCLE){
    ra -= RA_SEC_IN_FULL_CIRCLE;
  }


  return ra;
}


//get declination in minutes of arc
long getDecMin()
{
  long decAng = filtDec - DEC_ENC_ZERO;

  //subtract an adjustable offset to allow setting of circle
  long dec = (long)(  (long)((float)decAng * STEPS_TO_DEC_MIN) - decOffset  );

  //force angle in range.   
  while(dec < DEC_MIN_SEC) {
    dec += DEC_MIN_IN_FULL_CIRCLE;
  }

  //fix angles >= 360deg 
  while(dec >= DEC_MAX_SEC) {
    dec -= DEC_MIN_IN_FULL_CIRCLE;
  }


  return dec;  
}



//get hour angle in minutes of arc
long getHaMin()
{
  long polAng = filtRa - RA_ENC_ZERO;

  //subtract an adjustable offset to allow setting of circle
  long ha = (long)(  (long)((float)polAng * STEPS_TO_DEC_MIN) - haOffset  );

  //force angle in range.   
  while(ha < DEC_MIN_SEC) {
    ha += DEC_MIN_IN_FULL_CIRCLE;
  }

  //fix angles >= 360deg 
  while(ha >= DEC_MAX_SEC) {
    ha -= DEC_MIN_IN_FULL_CIRCLE;
  }


  return ha;  
}

//serve a captive portal webpage to allow basic readout of circles and manual re-setting of circles 
void handleRoot() 
{
 webServer.send(200, "text/html", s);
}

//subtract the requested RA setting from the current RA in order to adjust the offset and re-set the circles
void handleRaSet() 
{
 int ra_hr = webServer.arg("ra_hr").toInt();
 int ra_min = webServer.arg("ra_min").toInt();
 int ra_sec = webServer.arg("ra_sec").toInt();

 long raReq = (long)ra_hr * 3600 +  (long)ra_min * 60 +  (long)ra_sec;

 raOffset += getRaSec() - raReq;

 
 webServer.send(200, "text/html", s);
 #ifdef DEBUG
 Serial.println("raSet");
 #endif
}

//subtract the requested Dec from the current Dec in order to adjust the offset and re-set the circles
void handleDecSet() 
{
 int dec_deg = webServer.arg("dec_deg").toInt();
 int dec_min = webServer.arg("dec_min").toInt();

 long decReq =  (long)dec_deg * 60 +  (long)dec_min;

 decOffset += getDecMin() - decReq;

 webServer.send(200, "text/html", s);
 #ifdef DEBUG
 Serial.println("decSet");
 #endif
 
}

//respond to AJAX query from webpage with pre-formed string encapsulating the RA and Dec values 
//to be displayed on the page. The page is scripted to generate AJAX queries at about 3/second.
void handlePosRead()
{
  String ra_str;
  String ha_str;
  String dec_str;

  long ra = getRaSec();

  //convert sec to hr:min:sec
  int ra_hr = ra / 3600;
  int ra_min = (ra -  (long)ra_hr * 3600) / 60;
  int ra_sec = (ra -  (long)ra_hr * 3600 -  (long)ra_min * 60);

  ra_str = String(ra_hr) + "h " + String(ra_min) + "\' " + String(ra_sec) + "\"";


  long dec = getDecMin();

  //convert arcmin to deg:arcmin
  int dec_deg = dec / 60;
  int dec_min = (dec -  (long)dec_deg * 60);

  dec_str = String(dec_deg) + "&#176; " + String(dec_min) + "\'";


   long ha = getHaMin();

  //convert arcmin to deg:arcmin
  int ha_deg = ha / 60;
  int ha_min = (ha -  (long)ha_deg * 60);

  ha_str = String(ha_deg) + "&#176; " + String(ha_min) + "\'";
  
  
  webServer.send(200, "text/html", ra_str+"\n"+ha_str+"\n"+dec_str+"\n");
  #ifdef DEBUG
  Serial.println("posrd");
  #endif
}

void setup()
{
  #ifdef DEBUG
  Serial.begin(57600);
  Serial.println("\nESP8266 boot");
  #endif

  //enable the encoder inputs that are connected to boot-strapping pins and were kept
  //disabled during boot
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);

  WiFi.mode(WIFI_AP);

  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(WiFi_Access_Point_Name);


  
    // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request
  dnsServer.start(DNS_PORT, "*", ip);

  // replay to all requests with same HTML
  webServer.onNotFound(handleRoot);
  webServer.on("/", handleRoot);
  webServer.on("/set_ra", handleRaSet);
  webServer.on("/set_dec", handleDecSet);
  webServer.on("/pos_read", handlePosRead);
  
  webServer.begin();


  // tcp listener to receive incoming connections from Skysafari:
  server.begin();
  server.setNoDelay(true);

  startTime = millis();

}

void loop()
{


  attendTcpRequests();
  yield();
  dnsServer.processNextRequest();
  yield();
  webServer.handleClient();
  yield();

#ifdef DEBUG
  if (millis() - last_debug > DEBUG_PERIOD) // only take new debug measurements for printout if enough time has elapsed.
  {
    //various debug printout functions

    readEncoder();

    Serial.print("err:");
    #ifndef REVERT_TO_ORIGINAL_LIB
    Serial.print(raDecEnc.readErrorsA());
    Serial.print(",");
    Serial.println(raDecEnc.readErrorsB());
    #else
    Serial.print(raEnc.readErrors());
    Serial.print(",");
    Serial.println(decEnc.readErrors());
    #endif
    
    last_debug = millis();
  }
#endif

  //implement a simple lowpass filter in order to reject single-reading jumps of the encoder position.
  //With my encoders, does not seem to interpolate between readings (insufficient noise in encoder
  //position readout for oversampling/averaging to have any precision multiplying effect). However it
  //does very effectively gets rid of the back and and forth jitter near a transition caused by noise,
  //turning it into a smoother continuous transition from one state to the next.
  if(millis() - lastFilter > FILTER_PERIOD){
    #ifndef REVERT_TO_ORIGINAL_LIB
    filtRa = filtRa + raDecEnc.readA() - (filtRa >> FILTER_EXP);
    filtDec = filtDec + raDecEnc.readB() - (filtDec >> FILTER_EXP);
    #else
    filtRa = filtRa + raEnc.read() - (filtRa >> FILTER_EXP);
    filtDec = filtDec + decEnc.read() - (filtDec >> FILTER_EXP);
    #endif
    lastFilter = millis();
  }


  yield();  // altough ESP8266 is supposed to do its work background TCP/wiFi after each loop, yieldig here can't hurt
}

//debug function
void readEncoder()
{
  #ifndef REVERT_TO_ORIGINAL_LIB
  long newPosition = raDecEnc.readA();
  long newPosition2 = raDecEnc.readB();
  #else
  long newPosition = raEnc.read();
  long newPosition2 = decEnc.read();
  #endif
  if (newPosition != oldPosition || newPosition2 != oldPosition2) 
  {
    oldPosition = newPosition;
    oldPosition2 = newPosition2;
    Serial.print(newPosition);
    Serial.print(", ");
    Serial.println(newPosition2);
  }
}

void attendTcpRequests()
{
  // check for new or lost connections:
  if (server.hasClient())
  {
    #ifdef DEBUG
    Serial.println("hasClient!");
    #endif
    if (!remoteClient || !remoteClient.connected())
    {
      if (remoteClient)
      {
        #ifdef DEBUG
        Serial.print("Client Disconnected\n");
        #endif
        remoteClient.stop();
      }
      remoteClient = server.available();
      //Serial.print("Inbound connection from: ");
      //Serial.println(remoteClient.remoteIP());
      //  remoteClient.flush();
      remoteClient.setNoDelay(true);
    }
  }

  // when we have a new incoming connection from Skysafari:
  while (remoteClient.available())
  {
    byte skySafariCommand = remoteClient.read();

    if (skySafariCommand == 81)  // 81 is ascii for Q, which is the only command skysafari sends to "basic encoders"
    {
      char encoderResponse[20];

      //calculate absolute encoder positions if indices/absolute encoders are used (hard-coded calibration constant, 
      //to be tuned based on installed position of encoder on the mount).
      //the constants are automatically set to zero by #ifdef if indexing encoders are not used.
      long iRaReading = filtRa - RA_ENC_ZERO;
      long iDecReading = filtDec - DEC_ENC_ZERO;
      
      sprintf(encoderResponse, "%i\t%i\t\n", iRaReading, iDecReading);
    #ifdef DEBUG
      Serial.println(encoderResponse);
    #endif

      remoteClient.println(encoderResponse);
    }
    else if (skySafariCommand == 72) // 'H' - request for encoder resolution, e.g. 10000-10000\n
    {
      char response[20];
      // Resolution on both axis is equal
      snprintf(response, 20, "%u %u", STEPS_IN_FULL_CIRCLE, STEPS_IN_FULL_CIRCLE);
//      Serial.println(response);

      remoteClient.println(response);
    #ifdef DEBUG
      Serial.println(response);
    #endif
    
    }
    else
    {
    #ifdef DEBUG
      Serial.print("*****");
      Serial.println(skySafariCommand);
    #endif
    }
  }
}
