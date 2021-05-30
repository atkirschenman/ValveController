#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <SD.h> 
#include <SPI.h>
#include <ESP8266HTTPClient.h>
#include <SD.h> 
#include <SPI.h>
#include <Wire.h> 
#include "FS.h"
#include <DS3231.h>
#include <Adafruit_MCP23017.h>
//#define DEBUG_ESP_HTTP_SERVER
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Variable Declarations
////////////////////////////////////////////////////////////////////////////////////
bool vset;
bool vZone1On,vZone1Off, vZone2On,vZone2Off,vZone3On,vZone3Off;
bool vZone1Stat, vZone2Stat, vZone3Stat, vMainStat;
bool vZ1FB2, vZ1FB1, vZ2FB2, vZ2FB1, vZ3FB2, vZ3FB1;
bool vSwitch1active=0, vSwitch2active=0, vSwitch3active=0, lastswitch=0;
bool vZT1Active=0, vZT2Active=0,vZT3Active=0, vZT1Good=0, vZT2Good=0,vZT3Good=0;
bool VwateredToday1,VwateredToday2,VwateredToday3;
unsigned int vTimesWatered1,vTimesWatered2,vTimesWatered3,vsecsoff=0;
bool vdryTest1,vdryTest2,vdryTest3;
bool vDayStart,check=false;
unsigned long vZ1D, vZ2D, vZ3D, vSecsInSun, vTimeOffline=0, vOldMillis, vOldMillis2, vtimeOffMillis;
unsigned long check_wifi = 30000;
unsigned int vTimesReset;
int vGDD, vmaxTemp=0, vminTemp=100,vGDDtot, vGDDlast;
int vZ1W, vZ2W, vZ3W;
int vZone1Moisture, vZone2Moisture, vZone3Moisture;
int vmoist1, vmoist2, vmoist3, vmoistlevel1, vmoistlevel2, vmoistlevel3, vmoistlevelset1, vmoistlevelset2, vmoistlevelset3;
int vLightlevel;
bool vmoistSelect1[3], vmoistSelect2[3], vmoistSelect3[3];
String formattedDate, dayStamp, timeStamp;
int vhours, vmins, vsecs, vday, voldDay1,voldHour,voldmins, vYear,vMonth;
String solStat1, solStat2,solStat3;
String timerStat1,timerStat2,timerStat3;
String Z1Dstat1,Z1Dstat2,Z1Dstat3;
String Z1Tstat1,Z1Tstat2,Z1Tstat3;
byte oldSec, vDaysinAve;
float vMonthDaylight=0;
int vmoista1[20],vmoista2[20],vmoista3[20], mi1,mi2,mi3;
float vHoursInSun;
byte vprecipChance[12], vChanceofPrecip12=0,vChanceofPrecip6=0;
bool Century=false;
bool h12;
bool PM;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;
const size_t capacity = JSON_ARRAY_SIZE(12) + 12*JSON_OBJECT_SIZE(1) + 12*JSON_OBJECT_SIZE(2) + 156*JSON_OBJECT_SIZE(3) + 12*JSON_OBJECT_SIZE(28) + 12240;
char sPayload[capacity];
#define MAXT 12 //how many timers I have in the program (or more)
struct timer1 { //This structure defines a new variable type that consists on:
  boolean in;      //IN: a digital input, that starts the timing after it is at ON state;
  unsigned int pt; //PT: after PT-time, output Q shall trigger to ON state; (Preset Time)
  unsigned int et; //ET: this is the amount of time after IN has gone to ON; (Elapsed Time)
  boolean q;       //Q: the timer output, that would be ON after ET=PT
};
timer1 ton[MAXT];  //Create 6 timers
unsigned long lastTime; //it must be long to receive the output of millis()
unsigned long wdt;

struct weatherAPI{
int temperature;
int realFeel;
int windSpeed;
int windDirection;
int precipChance;
};

weatherAPI weatherArray[12];
//formatteddate, vHoursInSun,vChanceofPrecip6,vChanceofPrecip12, vGDD,vGDDtot,VwateredToday1,VwateredToday2,VwateredToday3,daylight%, hours,hoursaverage, vRestart,voffline;
 ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Define I/O
////////////////////////////////////////////////////////////////////////////////////

#define vZone1Tmr 0
#define vZone2Tmr 1
#define Z1Dur 3
#define Z2Dur 4
#define Z3Dur 5
#define moist1 6
#define moist2 7
#define moist3 8
#define dayDebounce 9
#define vZone3Tmr 10
#define pwrTON 11
#define vZone1ON_PIN 17
#define vZone1OFF_PIN 25
#define vZone2ON_PIN 16
#define vZone2OFF_PIN 4
#define vZone3ON_PIN 14
#define vZone3OFF_PIN 27
#define vZone1ON_FB 33
#define vZone1OFF_FB 32
#define vZone2ON_FB 26
#define vZone2OFF_FB 0
#define vZone3ON_FB 13
#define vZone3OFF_FB 12
#define vMainON_PIN 2
#define switch1 1
#define switch2 3
#define switch3 15
#define capmoist1 36
#define capmoist2 39
#define capmoist3 34
#define LDR 35
//#define vMISO 23
//#define SD_CS_pin 15
#define vMotorTime 10000


bool init1[MAXT];
unsigned long mark[MAXT];

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Function Declarations
////////////////////////////////////////////////////////////////////////////////////
void WaterON();
void WaterOFF();
void MainON();
void MainOFF();
void timers_manager();
void IO_Function();
void WaterTimers();
void handleZone2();
void handleZone3();
void handleRoot();
void handlesettings();
void handlezonecontrol();
void handleZ1D();
void handleZ1A();
void handleZ1W();
void handleZ2D();
void handleZ2A();
void handleZ2W();
void handleZ3D();
void handleZ3A();
void handleZ3W();
void handleZ1S();
void handleZ2S();
void handleZ3S();
void handleZ1P();
void handleZ2P();
void handleZ3P();
void handleLED();
void handleTime();
void StatusSend();
void Daylight();
//void moistureSensing();
void Resets();
void SD_file_download();
void SD_file_download2();
void File_Download();
void SaveSettings();
void ReadSettings();
void parseJSON();

const char* ssid = "Kirschenman_2.4";
const char* password = "Buddydog17";
const String endpoint = "http://dataservice.accuweather.com/forecasts/v1/hourly/12hour/335566?apikey=imM6Er88413swHlQ8lA4039wAaeyM9Gx&language=en-us&details=true&metric=false";
const String key = "imM6Er88413swHlQ8lA4039wAaeyM9Gx";
WiFiClient client;
ESP8266WebServer server(80);
Adafruit_MCP23017 mcp[2];
DS3231 Clock;
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Setup
////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:



pinMode(D3, OUTPUT);
//pinMode(D4, OUTPUT);


IPAddress local_IP(10, 95, 141, 149); // Set your server's fixed IP address here
IPAddress gateway(10, 95, 141, 97);    // Set your network Gateway usually your Router base address
IPAddress subnet(255, 255, 255, 0);   // Set your network sub-network mask here
IPAddress dns(10, 95, 141, 97);        // Set your network DNS usually your Router base address
Serial.begin(115200);



  WiFi.mode(WIFI_STA); //Connectto your wifi
  WiFi.begin(ssid, password);
WiFi.config(local_IP, gateway, subnet, dns);

  Serial.println("Connecting to ");
  Serial.print(ssid);
 
  //Wait for WiFi to connect
  while(WiFi.waitForConnectResult() != WL_CONNECTED){      
      Serial.print(".");
    }
    
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
   
   if (!SD.begin(D8)) { // see if the card is present and can be initialised. Wemos SD-Card CS uses D8 
    Serial.println(F("Card failed or not present, no SD Card data logging possible..."));
   }else Serial.println("Hurray!");
 
    
  server.on("/", handleRoot); //Which routine to handle at root location. This is display page
  server.on("/setzone1", handleLED); 
  server.on("/setzone2", handleZone2); 
  server.on("/setzone3", handleZone3); 
   server.on("/settings", handlesettings);
   server.on("/zonecontrol", handlezonecontrol);
   server.on("/setZ1A", handleZ1A);  
   server.on("/Z1Dfunction", handleZ1D); 
   server.on("/Z1Wfunction", handleZ1W); 
    server.on("/setZ2A", handleZ2A);  
   server.on("/Z2Dfunction", handleZ2D); 
   server.on("/Z2Wfunction", handleZ2W); 
    server.on("/setZ3A", handleZ3A);  
   server.on("/Z3Dfunction", handleZ3D); 
   server.on("/Z3Wfunction", handleZ3W); 
  server.on("/statusRead", StatusSend); 
  server.on("/Z1S", handleZ1S); 
  server.on("/Z2S", handleZ2S); 
  server.on("/Z3S", handleZ3S); 
  server.on("/Z1Pfunction", handleZ1P); 
  server.on("/Z2Pfunction", handleZ2P); 
  server.on("/Z3Pfunction", handleZ3P); 
  server.on("/DownloadSettings",SD_file_download2);
  server.on("/DownloadData",SD_file_download);
   
   server.begin(); //Start server
  Serial.println("HTTP server started");
  Wire.begin();
   //  if (!SD.begin(SD_CS_pin)) { // see if the card is present and can be initialised. Wemos SD-Card CS uses D8 
  //  Serial.println(F("Card failed or not present, no SD Card data logging possible..."));
  // }else Serial.println("Hurray!");

    if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
 // EEPROM.begin(40);//bytes
  //variables to save:Z1A,Z1D,Z1W,Z2A,Z2D,Z2W,Z3A,Z3D,Z3W,vmoistSelect1[1,2,3],vmoistSelect2[1,2,3],vmoistSelect3[1,2,3],
//  setWD();
//xTaskCreatePinnedToCore(watchDog, "watchdog", 512, NULL, 1, NULL, 0);

    
//     ArduinoOTA
//    .onStart([]() {
//      String type;
//      if (ArduinoOTA.getCommand() == U_FLASH)
//        type = "sketch";
//      else // U_SPIFFS
//        type = "filesystem";
//
//      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
//      Serial.println("Start updating " + type);
//    })
//    .onEnd([]() {
//      Serial.println("\nEnd");
//    })
//    .onProgress([](unsigned int progress, unsigned int total) {
//      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//    })
//    .onError([](ota_error_t error) {
//      Serial.printf("Error[%u]: ", error);
//      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//      else if (error == OTA_END_ERROR) Serial.println("End Failed");
//    });
//
//  ArduinoOTA.begin();

}

void loop() {
  //if (vOldMillis2+10<millis()){
  server.handleClient();
 // vOldMillis2=millis();
 // }
  // put your main code here, to run repeatedly:
if (vOldMillis+500<millis()){
 WaterTimers();
timers_manager();
WaterON();
server.handleClient();
IO_Function();
server.handleClient();
handleTime();
server.handleClient();
Resets();
vOldMillis=millis();
}
//if ((millis()-vOldMillis)>3000) {vTimesReset=8; vTimeOffline=millis()-vOldMillis; vOldMillis=millis();}
//moistureSensing();

//if ((millis()-vOldMillis)>3000) {vTimesReset=9; vTimeOffline=millis()-vOldMillis; vOldMillis=millis();}
//Daylight();


//ReconnectFunction();
//timerWrite(timer, 0); //reset timer (feed watchdog)
//Serial.print(ton[Z1Dur].q); Serial.print("\t");
//Serial.print(ton[Z2Dur].q); Serial.print("\t");
//Serial.print(ton[Z3Dur].q); Serial.print("\t");
//Serial.print(VwateredToday1); Serial.print("\t");
//Serial.print(VwateredToday2); Serial.print("\t");
//Serial.print(VwateredToday3); Serial.print("\t");
//Serial.print(ton[Z1Dur].in); Serial.print("\t");
//Serial.print(ton[Z2Dur].in); Serial.print("\t");
//Serial.print(ton[Z3Dur].in); Serial.print("\t");
//Serial.print(ton[Z1Dur].pt); Serial.print("\t");
//Serial.print(ton[Z2Dur].pt); Serial.print("\t");
//Serial.print(ton[Z3Dur].pt); Serial.print("\t");
//Serial.print(ton[Z3Dur].et); Serial.print("\t");
//Serial.print(ton[Z3Dur].et); Serial.print("\t");
//Serial.print(ton[Z3Dur].et); Serial.println("\t");

//vOldMillis=millis();
}





////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Root
////////////////////////////////////////////////////////////////////////////////////
void handleRoot() {
  Serial.println("root");
    String pathWithExtension = "/GardenStatus.html";
  if (SPIFFS.exists(pathWithExtension)) {
    Serial.println(F("D43 stylsheet found on SPIFFS"));
    File file = SPIFFS.open(pathWithExtension, "r");
    size_t sent = server.streamFile(file, "text/html");
    file.close();
  }
  else
  {
    Serial.println(F("D49 stylsheet not found on SPIFFS"));
    //handleNotFound;
  }
 //server.send(200, "text/html", "/GardenStatus.html"); //Send web page
}

void handlesettings() {
      String pathWithExtension = "/GardenSettings.html";
  if (SPIFFS.exists(pathWithExtension)) {
    Serial.println(F("D43 stylsheet found on SPIFFS"));
    File file = SPIFFS.open(pathWithExtension, "r");
    size_t sent = server.streamFile(file, "text/html");
    file.close();
  }
  else
  {
    Serial.println(F("D49 stylsheet not found on SPIFFS"));
    //handleNotFound;
  }
 //server.send(200, "text/html", "/GardenSettings.html"); //Send web page
}
void handlezonecontrol() {
      String pathWithExtension = "/GardenIndex.html";
  if (SPIFFS.exists(pathWithExtension)) {
    Serial.println(F("D43 stylsheet found on SPIFFS"));
    File file = SPIFFS.open(pathWithExtension, "r");
    size_t sent = server.streamFile(file, "text/html");
    file.close();
  }
  else
  {
    Serial.println(F("D49 stylsheet not found on SPIFFS"));
    //handleNotFound;
  }
 //server.send(200, "text/html", "/GardenIndex.html"); //Send web page
}
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone1
////////////////////////////////////////////////////////////////////////////////////

void handleLED() {
 String ledState = "OFF";
 String t_state = server.arg("Zone1"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 //Serial.println(t_state);
 if(t_state == "1")
 {
 vZone1On=1;
 }
 else
 {
  vZone1Off=1;
 }
 
 server.send(200, "text/plane", ledState); //Send web page
}

void handleZone2(){
   String ledState = "OFF";
 String t_state = server.arg("Zone2"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 //Serial.println(t_state);
 if(t_state == "1")
 {
 vZone2On=1;
 }
 else
 {
  vZone2Off=1;
 }
 
 server.send(200, "text/plane", ledState); //Send web page
}
void handleZone3(){
   String ledState = "OFF";
 String t_state = server.arg("Zone3"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 //Serial.println(t_state);
 if(t_state == "1")
 {
 vZone3On=1;
 }
 else
 {
  vZone3Off=1;
 }
 
 server.send(200, "text/plane", ledState); //Send web page
}
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone1 timer Active
////////////////////////////////////////////////////////////////////////////////////

void handleZ1A() {
 String Z1Astate = "OFF";
 String t_state = server.arg("Z1Astate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
// Serial.println(t_state);
 if(t_state == "1")
 {
// Serial.print("Active");
 vZT1Active=1;
 }
 else
 {
 //Serial.print("Not active");
 vZT1Active=0;
 }
 
 server.send(200, "text/plane", Z1Astate); //Send web   page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 1 Duration
////////////////////////////////////////////////////////////////////////////////////
void handleZ1D() {
 
 String t_state = server.arg("Z1Dfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 
  vZ1D=t_state.toInt();
  
 ton[Z1Dur].pt=vZ1D*60000;

 server.send(200, "text/plane", t_state); //Send web page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 1 Time of Day
////////////////////////////////////////////////////////////////////////////////////
void handleZ1W() {
 
 String t_state = server.arg("Z1Wfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 vZ1W=t_state.toInt();
 
  

 server.send(200, "text/plane", t_state); //Send web page
}


////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone2 timer Active
////////////////////////////////////////////////////////////////////////////////////

void handleZ2A() {
 String Z2Astate = "OFF";
 String t_state = server.arg("Z2Astate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
// Serial.println(t_state);
 if(t_state == "1")
 {
// Serial.print("Active");
 vZT2Active=1;
 }
 else
 {
 //Serial.print("Not active");
 vZT2Active=0;
 }
 
 server.send(200, "text/plane", Z2Astate); //Send web   page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 2 Duration
////////////////////////////////////////////////////////////////////////////////////
void handleZ2D() {
 
 String t_state = server.arg("Z2Dfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 
  vZ2D=t_state.toInt();
  
 ton[Z2Dur].pt=vZ2D*60000;

 server.send(200, "text/plane", t_state); //Send web page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 2 Time of Day
////////////////////////////////////////////////////////////////////////////////////
void handleZ2W() {
 
 String t_state = server.arg("Z2Wfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 vZ2W=t_state.toInt();
 
  

 server.send(200, "text/plane", t_state); //Send web page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone3 timer Active
////////////////////////////////////////////////////////////////////////////////////

void handleZ3A() {
 String Z3Astate = "OFF";
 String t_state = server.arg("Z3Astate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
// Serial.println(t_state);
 if(t_state == "1")
 {
// Serial.print("Active");
 vZT3Active=1;
 }
 else
 {
 //Serial.print("Not active");
 vZT3Active=0;
 }
 
 server.send(200, "text/plane", Z3Astate); //Send web   page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 3 Duration
////////////////////////////////////////////////////////////////////////////////////
void handleZ3D() {
 
 String t_state = server.arg("Z3Dfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 
  vZ3D=t_state.toInt();
  
 ton[Z3Dur].pt=vZ3D*60000;

 server.send(200, "text/plane", t_state); //Send web page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 3 Time of Day
////////////////////////////////////////////////////////////////////////////////////
void handleZ3W() {
 
 String t_state = server.arg("Z3Wfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 vZ3W=t_state.toInt();
 
  

 server.send(200, "text/plane", t_state); //Send web page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 1 Sensing
////////////////////////////////////////////////////////////////////////////////////
void handleZ1S(){
  int x=0;
   String t_state = server.arg("Z1Sfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 x=t_state.toInt();
if (x>=100){
  x-=100;
  vmoistSelect1[0]=1;
 }
 else vmoistSelect1[0]=0;
 
if (x>=10){
  x-=10;
  vmoistSelect1[1]=1;
 }
 else vmoistSelect1[1]=0;
  
if (x>=1){
  vmoistSelect1[2]=1;
 }
 else vmoistSelect1[2]=0;
  server.send(200, "text/plane", t_state); //Send web page
}


////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 2 Sensing
////////////////////////////////////////////////////////////////////////////////////
void handleZ2S(){
   int x=0;
   String t_state = server.arg("Z2Sfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 x=t_state.toInt();
if (x>=100){
  x-=100;
  vmoistSelect2[0]=1;
 }
 else vmoistSelect2[0]=0;
 
if (x>=10){
  x-=10;
  vmoistSelect2[1]=1;
 }
 else vmoistSelect2[1]=0;
  
if (x>=1){
  vmoistSelect2[2]=1;
 }
 else vmoistSelect2[2]=0;
  server.send(200, "text/plane", t_state); //Send web page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle Zone 3 Sensing
////////////////////////////////////////////////////////////////////////////////////
void handleZ3S(){
   int x=0;
   String t_state = server.arg("Z3Sfunc"); 
 x=t_state.toInt();
if (x>=100){
  x-=100;
  vmoistSelect3[0]=1;
 }
 else vmoistSelect3[0]=0;
 
if (x>=10){
  x-=10;
  vmoistSelect3[1]=1;
 }
 else vmoistSelect3[1]=0;
  
if (x>=1){
  vmoistSelect3[2]=1;
 }
 else vmoistSelect3[2]=0;
  server.send(200, "text/plane", t_state); //Send web page
}

void handleZ1P(){
     int x=0;
   String t_state = server.arg("Z1Pfunc"); 
 x=t_state.toInt();
 vmoistlevelset1=x;
  server.send(200, "text/plane", t_state); //Send web page
}
void handleZ2P(){
   int x=0;
   String t_state = server.arg("Z2Pfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
  x=t_state.toInt();
  vmoistlevelset2=x;
   server.send(200, "text/plane", t_state); //Send web page
}
void handleZ3P(){
   int x=0;
   String t_state = server.arg("Z3Pfunc"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
  x=t_state.toInt();
  vmoistlevelset3=x;
   server.send(200, "text/plane", t_state); //Send web page
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Status Send
////////////////////////////////////////////////////////////////////////////////////
void StatusSend(){

String t12hr1=" ",t12hr2=" ",t12hr3=" ";
if (vZone1Stat) solStat1="ON";
else solStat1="OFF";
if (vZone2Stat) solStat2="ON";
else solStat2="OFF";
if (vZone3Stat) solStat3="ON";
else solStat3="OFF";
if (vZT1Active) timerStat1="ON";
else timerStat1="OFF";
if (vZT2Active) timerStat2="ON";
else timerStat2="OFF";
if (vZT3Active) timerStat3="ON";
else timerStat3="OFF";

if (vZ1W>12){
  t12hr1+= (vZ1W-12);
  t12hr1+=" pm";
}
else{
  t12hr1+=vZ1W;
  t12hr1+=" am";
}

if (vZ2W>12){
  t12hr2+= (vZ2W-12);
  t12hr2+=" pm";
}
else{
  t12hr2+=vZ2W;
  t12hr2+=" am";
}

if (vZ3W>12){
  t12hr3+= (vZ3W-12);
  t12hr3+=" pm";
}
else{
  t12hr3+=vZ3W; 
  t12hr3+=" am";
}


String readings= "[ \" ";
readings+=solStat1;
readings+=" \", \" ";
readings+=solStat2;
readings+=" \", \" ";
readings+=solStat3;
readings+=" \", \" ";
readings+=timerStat1;
readings+=" \", \" ";
readings+=vZ1D;
readings+=" \", \" ";
readings+=t12hr1;
readings+=" \", \" ";
readings+=timerStat2;
readings+=" \", \" ";
readings+=vZ2D;
readings+=" \", \" ";
readings+=t12hr2;
readings+=" \", \" ";
readings+=timerStat3;
readings+=" \", \" ";
readings+=vZ3D;
readings+=" \", \" ";
readings+=t12hr3;
readings+=" \", \" ";
readings+=vmoistlevel1;
readings+=" \", \" ";
readings+=vmoistlevel2;
readings+=" \", \" ";
readings+=vmoistlevel3;
readings+=" \", \" ";
readings+=vZone1Moisture;
readings+=" \", \" ";
readings+=vZone2Moisture;
readings+=" \", \" ";
readings+=vZone3Moisture;
readings+=" \", \" ";
readings+=vmoistlevelset1;
readings+=" \", \" ";
readings+=vmoistlevelset2;
readings+=" \", \" ";
readings+=vmoistlevelset3;
readings+=" \", \" ";
readings+=vLightlevel;
readings+=" \", \" ";
readings+=vHoursInSun;
readings+=" \", \" ";
readings+=vMonthDaylight;
readings+=" \", \" ";
readings+=vGDD;
readings+=" \", \" ";
readings+=vGDDlast;
readings+=" \", \" ";
readings+=vGDDtot;
readings+=" \", \" ";
readings+=vprecipChance[0];
readings+=" \", \" ";
readings+=vChanceofPrecip6;
readings+=" \", \" ";
readings+=vChanceofPrecip12;
readings+=" \", \" ";
readings+=vTimesReset;
readings+=" \", \" ";
readings+=vTimeOffline;
readings+=" \", \" ";
readings+=vMonth;
readings+="/";
readings+=vday;
readings+="/";
readings+=vYear;
readings+=" ";
readings+=vhours;
readings+=":";
readings+=vmins;
readings+=":";
readings+=vsecs;
readings+="\" ]";


  server.send(200, "text/plane", readings); //Send web page
}





void SD_file_download(){

    File download = SD.open("/Data.csv");
    if (download) {
      Serial.println("here");
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=Data.csv");
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
 
  } 
}

void SD_file_download2(){
    SaveSettings();
    File download = SD.open("/Settings.csv");
    if (download) {
      Serial.println("here");
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=Settings.csv");
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
 
  } 
}
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Write CSV
////////////////////////////////////////////////////////////////////////////////////
void SaveData(){
  //formatteddate, vHoursInSun,vChanceofPrecip6,vChanceofPrecip12, vGDD,vGDDtot,VwateredToday1,VwateredToday2,VwateredToday3,vLightlevel, hours,hoursaverage, vRestart,voffline;
   String dataString="";
  dataString+=vMonth;
  dataString+="/";
  dataString+=vday;
  dataString+="/";
  dataString+=vYear;
  dataString+=" ";
  dataString+=vhours;
  dataString+=":";
  dataString+=vmins;
  dataString+=":";
  dataString+=vsecs;
   dataString+=",";
   dataString+=vHoursInSun;
   dataString+=",";
   dataString+=vChanceofPrecip6;
   dataString+=",";
   dataString+=vChanceofPrecip12;
   dataString+=",";
   dataString+=vGDD;
   dataString+=",";
   dataString+=vGDDtot;
   dataString+=",";
   dataString+=vTimesWatered1;
   dataString+=",";
   dataString+=vTimesWatered2;
   dataString+=",";
   dataString+=vTimesWatered3;
   dataString+=",";
   dataString+=vLightlevel;
   dataString+=",";
   dataString+=vHoursInSun;
   dataString+=",";
   dataString+=vMonthDaylight;
   dataString+=",";
   dataString+=vTimesReset;
   dataString+=",";
   dataString+=vTimeOffline;
   dataString+=",";
   dataString+=vmoistlevel1;
   dataString+=",";
   dataString+=vmoistlevel2;
   dataString+=",";
   dataString+=vmoistlevel3;
  
   File logFile = SD.open("/Data.csv", "a"); 
  

   if (logFile){
    
    logFile.println(dataString);
    logFile.close();
    Serial.println(dataString);
  }

  else {
    Serial.println("Couldn't open log file");
  }
}
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Write Settings
////////////////////////////////////////////////////////////////////////////////////
void SaveSettings(){
//  dataString=String(vZT1Active);//Zone 1 timer
//  dataString+=",";
//   dataString+=vZ1D;
//   dataString+=",";
//   dataString+=vZ1W;
//   dataString+=",";
//   dataString+=vZT2Active;//Zone 2 timer
//   dataString+=",";
//   dataString+=vZ2D;
//   dataString+=",";
//   dataString+=vZ2W;
//   dataString+=",";
//   dataString+=vZT3Active;//Zone 3 timer
//   dataString+=",";
//   dataString+=vZ3D;
//   dataString+=",";
//   dataString+=vZ3W;
//   dataString+=",";
//   dataString+=vmoistSelect1[0];//vmoistSelect1
//   dataString+=",";
//   dataString+=vmoistSelect1[1];
//   dataString+=",";
//   dataString+=vmoistSelect1[2];
//   dataString+=","; 
//   dataString+=vmoistSelect2[0];//vmoistSelect2
//   dataString+=",";
//   dataString+=vmoistSelect2[1];
//   dataString+=",";
//   dataString+=vmoistSelect2[2];
//   dataString+=",";
//   dataString+=vmoistSelect3[0];//vmoistSelect3
//   dataString+=",";
//   dataString+=vmoistSelect3[1];
//   dataString+=",";
//   dataString+=vmoistSelect3[2];
//      dataString+=",";
//   dataString+=vmoistlevelset1;
//      dataString+=",";
//   dataString+=vmoistlevelset2;
//      dataString+=",";
//   dataString+=vmoistlevelset3;
//   dataString+=",";
//   dataString+=vMonthDaylight;
//   dataString+=",";
//   dataString+=voldDay1;
//    dataString+=",";
//   dataString+=voldHour;
//   dataString+=",";
//   dataString+=vTimesReset;
//   dataString+=",";
//   dataString+=vTimeOffline;
//   dataString+=",";
//   dataString+=vGDD;
//   dataString+=",";
//   dataString+=vGDDtot;
//   dataString+=",";
//   dataString+=vTimesWatered1;//vmoistSelect3
//   dataString+=",";
//   dataString+=vTimesWatered2;
//   dataString+=",";
//   dataString+=vTimesWatered3;
//   dataString+=",";
//   dataString+=VwateredToday1;
//   dataString+=",";
//   dataString+=VwateredToday2;
//   dataString+=",";
//   dataString+=VwateredToday3;
  String dataString="";
  //variables to save:Z1A,Z1D,Z1W,Z2A,Z2D,Z2W,Z3A,Z3D,Z3W,vmoistSelect1[1,2,3],vmoistSelect2[1,2,3],vmoistSelect3[1,2,3],vMonthDaylight
  dataString=String(vZT1Active);//Zone 1 timer
  dataString+=",";
   dataString+=vZ1D;
   dataString+=",";
   dataString+=vZ1W;
   dataString+=",";
   dataString+=vZT2Active;//Zone 2 timer
   dataString+=",";
   dataString+=vZ2D;
   dataString+=",";
   dataString+=vZ2W;
   dataString+=",";
   dataString+=vZT3Active;//Zone 3 timer
   dataString+=",";
   dataString+=vZ3D;
   dataString+=",";
   dataString+=vZ3W;
   dataString+=",";
   dataString+=vmoistSelect1[0];//vmoistSelect1
   dataString+=",";
   dataString+=vmoistSelect1[1];
   dataString+=",";
   dataString+=vmoistSelect1[2];
   dataString+=","; 
   dataString+=vmoistSelect2[0];//vmoistSelect2
   dataString+=",";
   dataString+=vmoistSelect2[1];
   dataString+=",";
   dataString+=vmoistSelect2[2];
   dataString+=",";
   dataString+=vmoistSelect3[0];//vmoistSelect3
   dataString+=",";
   dataString+=vmoistSelect3[1];
   dataString+=",";
   dataString+=vmoistSelect3[2];
      dataString+=",";
   dataString+=vmoistlevelset1;
      dataString+=",";
   dataString+=vmoistlevelset2;
      dataString+=",";
   dataString+=vmoistlevelset3;
   dataString+=",";
   dataString+=vMonthDaylight;
   dataString+=",";
   dataString+=voldDay1;
    dataString+=",";
   dataString+=voldHour;
   dataString+=",";
   dataString+=vTimesReset;
   dataString+=",";
   dataString+=vTimeOffline;
   dataString+=",";
   dataString+=vGDD;
   dataString+=",";
   dataString+=vGDDtot;
   dataString+=",";
   dataString+=vTimesWatered1;//vmoistSelect3
   dataString+=",";
   dataString+=vTimesWatered2;
   dataString+=",";
   dataString+=vTimesWatered3;
   dataString+=",";
   dataString+=VwateredToday1;
   dataString+=",";
   dataString+=VwateredToday2;
   dataString+=",";
   dataString+=VwateredToday3;
  
   File logFile = SD.open("/Settings.csv", "w"); 

  if (logFile){
    
    logFile.println(dataString);
    logFile.close();
    Serial.println(dataString);
  }

  else {
    Serial.println("Couldn't open log file");
  }
  ton[Z1Dur].pt=vZ1D*60000;
ton[Z2Dur].pt=vZ2D*60000;
ton[Z3Dur].pt=vZ3D*60000;
}

void ReadSettings(){

 handleTime();
 //EEPROM
// vZT1Active=s.toInt();
//          vZ1D=s.toInt();
//          vZ1W=s.toInt();
//          vZT2Active=s.toInt();
//          vZ2D=s.toInt();
//          vZ2W=s.toInt();
//          vZT3Active=s.toInt();
//          vZ3D=s.toInt();
//          vZ3W=s.toInt();
//          vmoistSelect1[0]=s.toInt();
//          vmoistSelect1[1]=s.toInt();
//          vmoistSelect1[2]=s.toInt();
//          vmoistSelect2[0]=s.toInt();
//          vmoistSelect2[1]=s.toInt();
//          vmoistSelect2[2]=s.toInt();
//          vmoistSelect3[0]=s.toInt();
//          vmoistSelect3[1]=s.toInt();
//          vmoistSelect3[2]=s.toInt();
//          vmoistlevelset1=s.toInt();
//          vmoistlevelset2=s.toInt();
//          vmoistlevelset3=s.toInt();
//          vMonthDaylight=s.toFloat() ;
//          voldDay1=s.toInt();
//          voldHour=s.toInt();
//          vTimesReset=s.toInt();
//          vTimeOffline=s.toInt();
//          vGDD=s.toInt();
//          vGDDtot=s.toInt();
//          vTimesWatered1=s.toInt();
//          vTimesWatered2=s.toInt();
//          vTimesWatered3=s.toInt();
//          VwateredToday1=s.toInt();
//          VwateredToday2=s.toInt();
//          VwateredToday3=s.toInt();
 
 File logFile = SD.open("/Settings.csv", "r"); 

  if (logFile){
    int i=1;
    char c;
    String s;
    while((c=logFile.read())!=-1 &&i<35){
      
      if (c!=',' && c!='\n') s+=c;
      else{
        switch (i){
          case 1:
          vZT1Active=s.toInt();
          break;
          case 2:
          vZ1D=s.toInt();
          break;
          case 3:
          vZ1W=s.toInt();
          break;
          case 4:
          vZT2Active=s.toInt();
          break;
          case 5:
          vZ2D=s.toInt();
          break;
          case 6:
          vZ2W=s.toInt();
          break;
          case 7:
          vZT3Active=s.toInt();
          break;
          case 8:
          vZ3D=s.toInt();
          break;
          case 9:
          vZ3W=s.toInt();
          break;
          case 10:
          vmoistSelect1[0]=s.toInt();
          break;
          case 11:
          vmoistSelect1[1]=s.toInt();
          break;
          case 12:
          vmoistSelect1[2]=s.toInt();
          break;
          case 13:
          vmoistSelect2[0]=s.toInt();
          break;
          case 14:
          vmoistSelect2[1]=s.toInt();
          break;
          case 15:
          vmoistSelect2[2]=s.toInt();
          break;
          case 16:
          vmoistSelect3[0]=s.toInt();
          break;
          case 17:
          vmoistSelect3[1]=s.toInt();
          break;
          case 18:
          vmoistSelect3[2]=s.toInt();
          break;
          case 19:
          vmoistlevelset1=s.toInt();
          break;
          case 20:
          vmoistlevelset2=s.toInt();
          break;
          case 21:
          vmoistlevelset3=s.toInt();
          break;
          case 22:
          vMonthDaylight=s.toFloat() ;
          break;
          case 23:
          voldDay1=s.toInt();
          break;
          case 24:
          voldHour=s.toInt();
          break;
          case 25:
          vTimesReset=s.toInt();
          break;
          case 26:
          vTimeOffline=s.toInt();
          break;
         case 27:
          vGDD=s.toInt();
          break;
          case 28:
          vGDDtot=s.toInt();
          break;
          case 29:
          vTimesWatered1=s.toInt();
          break;
          case 30:
          vTimesWatered2=s.toInt();
          break;
          case 31:
          vTimesWatered3=s.toInt();
          break;
          case 32:
          VwateredToday1=s.toInt();
          break;
          case 33:
          VwateredToday2=s.toInt();
          break;
          case 34:
          VwateredToday3=s.toInt();
          break;
          default:
          break;
        }
        i++;
        Serial.println(i);
       s="";
      }
    }
    logFile.close();
  }
Serial.print(vZT1Active); Serial.print("\t");
Serial.print(vZ1D); Serial.print("\t");
Serial.print(vZ1W); Serial.print("\t");
Serial.print(vZT2Active); Serial.print("\t");
Serial.print(vZ2D); Serial.print("\t");
Serial.print(vZ2W); Serial.print("\t");
Serial.print(vZT3Active); Serial.print("\t");
Serial.print(vZ3D); Serial.print("\t");
Serial.print(vZ3W); Serial.print("\t");
Serial.print(vmoistSelect1[0]); Serial.print("\t");
Serial.print(vmoistSelect1[1]); Serial.print("\t");
Serial.print(vmoistSelect1[2]); Serial.print("\t");
Serial.print(vmoistSelect2[0]); Serial.print("\t");
Serial.print(vmoistSelect2[1]); Serial.print("\t");
Serial.print(vmoistSelect2[2]); Serial.print("\t");
Serial.print(vmoistSelect3[0]); Serial.print("\t");
Serial.print(vmoistSelect3[1]); Serial.print("\t");
Serial.print(vmoistSelect3[2]); Serial.print("\t");
Serial.print(voldDay1); Serial.print("\t");
Serial.print(voldHour); Serial.print("\t");
Serial.print(vMonthDaylight); Serial.println("\t");
}




void IO_Function(){
  //Digital Writing vzone1Stat=1;
  //ton[vZone1Tmr].in=1;
  /*
  if (vZone1Stat &&ton[vZone1Tmr].in==1&&!ton[vZone1Tmr].q==1 ) digitalWrite(vZone1ON_PIN,HIGH);
  else digitalWrite(vZone1ON_PIN,LOW);
   if (!vZone1Stat &&ton[vZone1Tmr].in==1&&!ton[vZone1Tmr].q==1 ) digitalWrite(vZone1OFF_PIN,HIGH);
  else digitalWrite(vZone1OFF_PIN,LOW);
  if (vZone2Stat &&ton[vZone2Tmr].in==1&&!ton[vZone2Tmr].q==1 ) digitalWrite(vZone2ON_PIN,HIGH);
  else digitalWrite(vZone2ON_PIN,LOW);
   if (!vZone2Stat &&ton[vZone2Tmr].in==1&&!ton[vZone2Tmr].q==1 ) digitalWrite(vZone2OFF_PIN,HIGH);
  else digitalWrite(vZone2OFF_PIN,LOW);
    if (vZone3Stat &&ton[vZone3Tmr].in==1&&!ton[vZone3Tmr].q==1 ) digitalWrite(vZone3ON_PIN,HIGH);
  else digitalWrite(vZone3ON_PIN,LOW);
   if (!vZone3Stat &&ton[vZone3Tmr].in==1&&!ton[vZone3Tmr].q==1 ) digitalWrite(vZone3OFF_PIN,HIGH);
  else digitalWrite(vZone3OFF_PIN,LOW);
     if (vMainStat ) digitalWrite(vMainON_PIN,HIGH);
  else digitalWrite(vMainON_PIN,LOW);

  //Digital Reading
  if (!digitalRead(switch1)) vSwitch1active=1;
  else vSwitch1active=0; 
    if (!digitalRead(switch2)) vSwitch2active=1;
  else vSwitch2active=0; 
    if (!digitalRead(switch3)) vSwitch3active=1;
  else vSwitch3active=0; 
  //Physical Switch Logic
  if ((lastswitch != vSwitch1active) && vSwitch1active==1){
  lastswitch=vSwitch1active;
  vZone1On=1;
  }
  else if (lastswitch!=vSwitch1active){
    vZone1Off=1;
     lastswitch=vSwitch1active;
  }
  */


ton[pwrTON].pt=5000;
ton[pwrTON].in=!ton[pwrTON].q;
if (ton[pwrTON].q) vset=!vset;
digitalWrite(D3,vset);
//digitalWrite(D4,vset);
Serial.println(vset);

}

void TimerConditions(){
  
if (vhours==vZ1W && vmins<15 && !VwateredToday1 && vdryTest1 && vChanceofPrecip6<50) vZT1Good=true;
else vZT1Good=false;

if (vhours==vZ2W  && vmins<15 && !VwateredToday2 && vdryTest2 && vChanceofPrecip6<50) vZT2Good=true;
else vZT2Good=false;

if (vhours==vZ3W  && vmins<15  && !VwateredToday3 && vdryTest3 && vChanceofPrecip6<50) vZT3Good=true;
else vZT3Good=false;

}
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Watering Timer Logic
////////////////////////////////////////////////////////////////////////////////////
void WaterTimers(){
 //TimerConditions();
    //Shut water off when end of timer and water is on
  if ( vZone1Stat && ton[Z1Dur].q){
    vZone1Off=1;
    ton[Z1Dur].in=0;
  }
  if (vZT1Active && vZT1Good){
    vZone1On=1;
    ton[Z1Dur].in=1;
    VwateredToday1=1;
    vTimesWatered1++;
  }

///////////////////////////////
    if ( vZone2Stat && ton[Z2Dur].q){
    vZone2Off=1;
    ton[Z2Dur].in=0;
  }
   if (vZT2Active && vZT2Good){
    vZone2On=1;
    ton[Z2Dur].in=1;
    VwateredToday2=1;
    vTimesWatered2++;
  }


  //////////////////////////
    if ( vZone3Stat && ton[Z3Dur].q){
    vZone3Off=1;
    ton[Z3Dur].in=0;
  }
   if (vZT3Active && vZT3Good){
    vZone3On=1;
    ton[Z3Dur].in=1;
   VwateredToday3=1;
   vTimesWatered3++;
  }

}
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Time Dependent resets
////////////////////////////////////////////////////////////////////////////////////
 //Reset Day
 void Resets(){
 
  
//      if (oldSec!=vsecs ){
//        
//     
//      
//      //if ((vsecs-oldSec)!=1 && (oldSec-vsecs)!=59){
////       if(voldHour!=vhours){ 
////              if (vhours<voldHour) vsecsoff=abs(voldHour-(vhours+23))*3600;
////              else vsecsoff=(vhours-voldHour)*3600;
////              }
////              
////      if(vmins!=voldmins){
////              if (vmins<voldmins) vsecsoff+=(60-voldmins)*60;
////              else vsecsoff+=(vmins-voldmins)*60;
////              }
////      if(vsecs!=oldSec){
////              if (vsecs<oldSec) vsecsoff+=(60-oldSec);
////              else vsecsoff+=(vsecs-oldSec);
////              }
//       
//     // }
//     
//        
//   
//
//      
////      Serial.print(vsecs); Serial.print("\t");
////      Serial.print(oldSec); Serial.print("\t");
////      Serial.print(vsecsoff); Serial.println("\t");
//      oldSec=vsecs;
//      vsecsoff=0;
//    }


    
 if(vday!=voldDay1){
 VwateredToday1=0;
 VwateredToday2=0;
 VwateredToday3=0;

 vMonthDaylight=vMonthDaylight + ((vHoursInSun- vMonthDaylight) / (vDaysinAve+1));
 vHoursInSun=0;
 vSecsInSun=0;
 vDayStart=0;
 vDaysinAve++;
 vGDDlast=vGDD;
 vGDDtot+=vGDD;
 voldDay1=vday;
 vmaxTemp=0;
 vminTemp=120;


 
 }
if (vhours!=voldHour) {
  voldHour=vhours;
 parseJSON(); 
  
SaveSettings();
  
  
SaveData();
 
}
//if (vmins!=voldmins){
//  SaveSettings();
//  vmins=voldmins;
//}

//seconds reset
 }






////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Timer Manager
////////////////////////////////////////////////////////////////////////////////////
void timers_manager() { //routine to manage all the timers
  byte i;

  // timers
  for (i=0;i<MAXT;i++) {
    if (!ton[i].in) { //if input is off
      ton[i].q = false; //put output off
      ton[i].et = 0; //preload the "elapsed time" with "preset time". Nota that the count is towards zero.
      init1[i]=0;
    }
    ton[i].q = (ton[i].in && (ton[i].et >= ton[i].pt)); //output is high when "et" arrives to zero (and "in" is 1)
  }
 
 
    for (i=0;i<MAXT;i++) // for each timer
      if (ton[i].in && (ton[i].et<ton[i].pt)){
        if (init1[i]==0){ mark[i]=millis();
        }
           ton[i].et=millis()-mark[i];
           init1[i]=1;
      }
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Handle General Time of Day
////////////////////////////////////////////////////////////////////////////////////
void handleTime(){
  vYear=Clock.getYear();
  vMonth=Clock.getMonth(Century);
  vday=Clock.getDate();
  vhours=Clock.getHour(h12, PM);
  vmins=Clock.getMinute();
  vsecs=Clock.getSecond();
  //Serial.print(vhours);Serial.print("\t");Serial.print(vmins);Serial.print("\t");Serial.println(vsecs);
}


void parseJSON(){
 DeserializationError error;
 DynamicJsonDocument doc(capacity);
    if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status
 
    HTTPClient http;

// Send request
http.useHTTP10(true);
http.begin(endpoint + key);
http.GET();

// Parse response

error=deserializeJson(doc, http.getStream());

// Read values
//Serial.println(doc["time"].as<long>());

// Disconnect
http.end();
  }
 
  //DynamicJsonDocument doc(capacity);
  //deserializeJson(doc,sPayload);
  //DeserializationError error=deserializeJson(doc,sPayload);
  if (error)
  {
    Serial.println("parseObject() failed");
    Serial.println(error.c_str());
    return;
  }

//JsonObject root = doc[0];

/*const char* root_0_DateTime = root_0["DateTime"]; // "2019-10-30T20:00:00-05:00"
long root_0_EpochDateTime = root_0["EpochDateTime"]; // 1572483600
int root_0_WeatherIcon = root_0["WeatherIcon"]; // 35
const char* root_0_IconPhrase = root_0["IconPhrase"]; // "Partly cloudy"
bool root_0_HasPrecipitation = root_0["HasPrecipitation"]; // false
bool root_0_IsDaylight = root_0["IsDaylight"]; // false

JsonObject root_0_Temperature = root_0["Temperature"];
int root_0_Temperature_Value = root_0_Temperature["Value"]; // 26
const char* root_0_Temperature_Unit = root_0_Temperature["Unit"]; // "F"
int root_0_Temperature_UnitType = root_0_Temperature["UnitType"]; // 18

JsonObject root_0_RealFeelTemperature = root_0["RealFeelTemperature"];
int root_0_RealFeelTemperature_Value = root_0_RealFeelTemperature["Value"]; // 20
const char* root_0_RealFeelTemperature_Unit = root_0_RealFeelTemperature["Unit"]; // "F"
int root_0_RealFeelTemperature_UnitType = root_0_RealFeelTemperature["UnitType"]; // 18

JsonObject root_0_WetBulbTemperature = root_0["WetBulbTemperature"];
int root_0_WetBulbTemperature_Value = root_0_WetBulbTemperature["Value"]; // 21
const char* root_0_WetBulbTemperature_Unit = root_0_WetBulbTemperature["Unit"]; // "F"
int root_0_WetBulbTemperature_UnitType = root_0_WetBulbTemperature["UnitType"]; // 18

JsonObject root_0_DewPoint = root_0["DewPoint"];
int root_0_DewPoint_Value = root_0_DewPoint["Value"]; // 10
const char* root_0_DewPoint_Unit = root_0_DewPoint["Unit"]; // "F"
int root_0_DewPoint_UnitType = root_0_DewPoint["UnitType"]; // 18

JsonObject root_0_Wind_Speed = root_0["Wind"]["Speed"];
float root_0_Wind_Speed_Value = root_0_Wind_Speed["Value"]; // 6.9
const char* root_0_Wind_Speed_Unit = root_0_Wind_Speed["Unit"]; // "mi/h"
int root_0_Wind_Speed_UnitType = root_0_Wind_Speed["UnitType"]; // 9

JsonObject root_0_Wind_Direction = root_0["Wind"]["Direction"];
int root_0_Wind_Direction_Degrees = root_0_Wind_Direction["Degrees"]; // 300
const char* root_0_Wind_Direction_Localized = root_0_Wind_Direction["Localized"]; // "WNW"
const char* root_0_Wind_Direction_English = root_0_Wind_Direction["English"]; // "WNW"

JsonObject root_0_WindGust_Speed = root_0["WindGust"]["Speed"];
float root_0_WindGust_Speed_Value = root_0_WindGust_Speed["Value"]; // 6.9
const char* root_0_WindGust_Speed_Unit = root_0_WindGust_Speed["Unit"]; // "mi/h"
int root_0_WindGust_Speed_UnitType = root_0_WindGust_Speed["UnitType"]; // 9

int root_0_RelativeHumidity = root_0["RelativeHumidity"]; // 51

JsonObject root_0_Visibility = root_0["Visibility"];
int root_0_Visibility_Value = root_0_Visibility["Value"]; // 10
const char* root_0_Visibility_Unit = root_0_Visibility["Unit"]; // "mi"
int root_0_Visibility_UnitType = root_0_Visibility["UnitType"]; // 2

JsonObject root_0_Ceiling = root_0["Ceiling"];
long root_0_Ceiling_Value = root_0_Ceiling["Value"]; // 44000
const char* root_0_Ceiling_Unit = root_0_Ceiling["Unit"]; // "ft"
int root_0_Ceiling_UnitType = root_0_Ceiling["UnitType"]; // 0

int root_0_UVIndex = root_0["UVIndex"]; // 0
const char* root_0_UVIndexText = root_0["UVIndexText"]; // "Low"
vprecipChance[0] = root_0["PrecipitationProbability"]; // 0
int root_0_RainProbability = root_0["RainProbability"]; // 0
int root_0_SnowProbability = root_0["SnowProbability"]; // 0
int root_0_IceProbability = root_0["IceProbability"]; // 0

JsonObject root_0_TotalLiquid = root_0["TotalLiquid"];
int root_0_TotalLiquid_Value = root_0_TotalLiquid["Value"]; // 0
const char* root_0_TotalLiquid_Unit = root_0_TotalLiquid["Unit"]; // "in"
int root_0_TotalLiquid_UnitType = root_0_TotalLiquid["UnitType"]; // 1

JsonObject root_0_Rain = root_0["Rain"];
int root_0_Rain_Value = root_0_Rain["Value"]; // 0
const char* root_0_Rain_Unit = root_0_Rain["Unit"]; // "in"
int root_0_Rain_UnitType = root_0_Rain["UnitType"]; // 1

JsonObject root_0_Snow = root_0["Snow"];
int root_0_Snow_Value = root_0_Snow["Value"]; // 0
const char* root_0_Snow_Unit = root_0_Snow["Unit"]; // "in"
int root_0_Snow_UnitType = root_0_Snow["UnitType"]; // 1

JsonObject root_0_Ice = root_0["Ice"];
int root_0_Ice_Value = root_0_Ice["Value"]; // 0
const char* root_0_Ice_Unit = root_0_Ice["Unit"]; // "in"
int root_0_Ice_UnitType = root_0_Ice["UnitType"]; // 1

int root_0_CloudCover = root_0["CloudCover"]; // 37
const char* root_0_MobileLink = root_0["MobileLink"]; // "http://m.accuweather.com/en/us/brookings-sd/57006/hourly-weather-forecast/335566?day=1&hbhhour=20&lang=en-us"
const char* root_0_Link = root_0["Link"]; // "http://www.accuweather.com/en/us/brookings-sd/57006/hourly-weather-forecast/335566?day=1&hbhhour=20&lang=en-us"*/

JsonObject root;
for (int i =0; i<=11; i++){
root = doc[i];
weatherArray[i].precipChance=root["UVIndex"]["PrecipitationProbability"];
weatherArray[i].realFeel=root["RealFeelTemperature"]["Value"];
weatherArray[i].temperature=root["Temperature"]["Value"];
weatherArray[i].windDirection=root["Wind"]["Direction"]["Degrees"];
weatherArray[i].windSpeed=root["Wind"]["Speed"]["Value"];
}
//Serial.print(weatherArray[0].precipChance); Serial.print("\t");Serial.print(weatherArray[0].realFeel); Serial.print("\t");Serial.print(weatherArray[0].temperature); Serial.print("\t");Serial.print(weatherArray[0].windDirection); Serial.print("\t");Serial.println(weatherArray[0].windSpeed);


vChanceofPrecip12=0;
vChanceofPrecip6=0;
for (int i=6; i <=11; i++){
  if (vChanceofPrecip12<weatherArray[i].precipChance) vChanceofPrecip12=weatherArray[i].precipChance;
}
for (int i=0; i <=5; i++){
  if (vChanceofPrecip6<weatherArray[i].precipChance) vChanceofPrecip6=weatherArray[i].precipChance;
}
if(weatherArray[0].temperature >vmaxTemp) vmaxTemp=weatherArray[0].temperature;
if (weatherArray[0].temperature<vminTemp)vminTemp=weatherArray[0].temperature;
if (vmaxTemp>86) vmaxTemp=86;
if (vminTemp<50) vminTemp=50;

//GDD or GDU = (Daily Maximum Air Temperature + Daily Minimum Temperature)/2 – 50
vGDD=((vmaxTemp+vminTemp)/2)-50;
if (vGDD<0) vGDD=0;


}



void ReconnectFunction() {
  // if wifi is down, try reconnecting every 30 seconds
  if ((WiFi.status() != WL_CONNECTED) && (millis() > check_wifi)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    check_wifi = millis() + 30000;
  }
}

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Water On Function
////////////////////////////////////////////////////////////////////////////////////
void WaterON()
{
ton[vZone1Tmr].pt=vMotorTime;
ton[vZone1Tmr].pt=vMotorTime;
//vZ1FB2, vZ1FB1, vZ2FB2, vZ2FB1, vZ3FB2, vZ3FB1;

  //Turn on Zone 1//////////////
  if (vZone1On==1 && ton[vZone1Tmr].in==0){
  vZone1Stat=1;
  ton[vZone1Tmr].in=1;
  vZone1On=0;
  } else if (vZone1Off==1 && ton[vZone1Tmr].in==0){
  vZone1Stat=0;
  ton[vZone1Tmr].in=1;
  vZone1Off=0;
  }
  //turn on Zone 2//////////////////////
  if (vZone2On==1 && ton[vZone2Tmr].in==0){
  vZone2Stat=1;
  ton[vZone2Tmr].in=1;
  vZone2On=0;
  } else if (vZone2Off==1 && ton[vZone2Tmr].in==0){
  vZone2Stat=0;
  ton[vZone2Tmr].in=1;
  vZone2Off=0;
  }
  //Turn on Zone 3 //////////////
  if (vZone3On==1 && ton[vZone3Tmr].in==0){
  vZone3Stat=1;
  ton[vZone3Tmr].in=1;
  vZone3On=0;
  } else if (vZone3Off==1 && ton[vZone3Tmr].in==0){
  vZone3Stat=0;
  ton[vZone3Tmr].in=1;
  vZone3Off=0;
  }

  
  if (vZone1Stat||vZone2Stat||vZone3Stat) vMainStat=1;
  else vMainStat=0;
}

