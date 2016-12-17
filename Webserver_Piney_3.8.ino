/*December 11
  3.8 revamps the writeclient() function to mostly comply with html5
      adds traps for bogus bmp280 readings
     fixed problem with ios not opening the page because of missing !DOCTYPE
     adds monthly rainfall total
     main loop runs about 60 cycles per second on WeMos
  3.7 adds time sync from library example TimeNTP_ESP8266WiFi
      adds rainfall detection
      Rainfall bucket dump sensor between D2 and ground
  
  3.6  This version replaces mechanical contacts with Hall effect Sensor
       Anemometer switch goes between D3 and Ground
       I just placed the hall effectr sensor in place of the old switch
  Heavy lifting code borrowed from:
  http://www.instructables.com/id/Control-ESP8266-Over-the-Internet-from-Anywhere/
  BMP280 from https://github.com/mahfuz195/BMP280-Arduino-Library
  https://diyhacking.com/arduino-hall-effect-sensor-tutorial/
  */

#include <ESP8266WiFi.h>
#include <TimeLib.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include "BMP280.h"
#define P0 1013.25
BMP280 bmp;

const char* ssid = "Piney_Woods";
const char* password = "1mushroom";
const char* host = "192.168.0.8"; //lan ip address will be in serial printed - don't think this is neeced
const int  buttonPin = D2;    // the pin that the rain guagae
const int contactPin = D3;     // the number of the anemometer reading pin

int contactState = 0;
int lastreading = 0;
int buttonState = 0;         // current state of the raingauge
int lastButtonState = 0;     // previous state of the raingauge
int teeterCounter=0;         // counts bucket dumps

float ws, ws_old, maxws = 0 ;
float et = 0;                                 // if et is  int you will get divide by 0 errors.
// blips are consecutive contact closings
//lastreading used to ignore long keydown times.
double blip1 = 0;
double blip2 = 0;
double T, Ttemp, P=965, Ptemp=965,  maxP = 0; //yminT is yesterdays minimum temp
double minP = 100;
String stringHour, stringMinute, stringTime, upTime ;
// Hardcode following variables to maintain continuity after restart
float rainfall = 0, mrainfall= .75;
float yrainfall = 0;

double minT=100, maxT=41.5;
double yminT=16.45, ymaxT=38.41 ;
String minTTime="5:00 p.m.", maxTTime ="1:00 p.m.",  yminTTime="5:00 a.m.", ymaxTTime="2:00 p.m.", maxWTime;

WiFiServer server(80); 
// NTP Servers:
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov


const int timeZone = -8;     // pst
//const int timeZone = -7;  // daylight savings time

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
//const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
//byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

void setup() {
   pinMode(BUILTIN_LED, OUTPUT);
  // initialize the contact pin as an input:
  pinMode(contactPin, INPUT);  // what about INPUT_PULLUP?
  
  Serial.begin(115200);
  delay(10);

   // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
//end WiFi Connect
  
  Serial.println(WiFi.localIP());
  // initialize the LED pin as an output:

   
  pinMode(buttonPin, INPUT_PULLUP);  // initialize the rainguage pin as a input:
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
   setSyncProvider(getNtpTime);



 
//  setTime(23, 9, 0, 2, 12, 16); // set time to 23:09 2 December 2016     replaced by NTP server
 
  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());
 
  
  // BMP280 next 7 lines
  if (!bmp.begin()) {
    Serial.println("BMP init failed!");
     while (1);  // comment out this line if no BMP280 connected
  }
  else Serial.println("BMP init success!");

  bmp.setOversampling(4);

upTime=String(month())+"/"+String(day())+"/"+String((year())-2000);

}  //end setup()

void loop() {
  int counter = 0;
  char result = bmp.startMeasurment();

                               // if (millis() % 2000 ==0){   // only measure temperature every 2 seconds
 if (result != 0) {                                     
    delay(result);
    result = bmp.getTemperatureAndPressure(Ttemp, Ptemp);
  }  //end if result...
 
 
//if (counter % 60 ==0 ){
                                //  T=(1.8*Ttemp)+32;
                                
  
                                //  T=(.556*(T-32));                       // Convert old Temp reading back to Centigrade
 if (Ttemp >-50 and Ttemp <50){            /////  next few lines trap bogus bmp280 readings ... first set a reasonable range of temps
//                                            ////   then set the range of change to .25 F per cycle
                              // if (Ttemp - T > .14){   //T is the previous reading ...  .14C is about .25F  ...this ignores high readings
                              //    Ttemp=T;   // if the change in T is > .14, ignore it
                              //  }
                              //  if (Ttemp - T < -.14){   //if the T drops more than .14C ignore it
                              //    Ttemp=T;
                              //  }                        // rate of change stuff needs more work to ignore startup T
                              //  
                              //  else {
  T=Ttemp; //} // new T = Ttemp, 
  }  //  end if within -50 to 50
                              // } // end if (millis() % 10000 ==0){ 
 

if (Ptemp >758 and Ptemp <1391){      //   ignore unreasonable pressure readings
  P = Ptemp;}


  if (hour() == 0) {
    stringHour = "12"; // if not there 12:01 am becomes 0:01 
  }
  else{
  if (hour() > 12) {
	stringHour =  String(hour() - 12);
  }  //end if hour()>12
  
  if (hour()<13){
  stringHour =  String(hour());				
  }  // end if hour()<13
 
  }  // end else

  
  stringMinute =  String(minute());
  
  if (minute() < 10) {
    stringTime = stringHour + ":0" + stringMinute;
  }
  else {
    stringTime = stringHour + ":" + stringMinute;
  }

  if ( hour() < 12) {
    stringTime = stringTime  + " a.m. ";
  }
  else {
    stringTime = stringTime + " p.m. ";
  }


  T = (1.8 * T) + 32; // Convert Centigrade to Farenheit
  P = (P * 0.02953) + 2;  //Convert to in hg with fudge factor

  //save min and max values...

  if (second() % 2 ==0){
  if ( T > maxT) {
    maxT = T;
    maxTTime = stringTime;
  }
  if ( T < minT) {
    minT = T;
    minTTime = stringTime;
  }
 }  // end if(second() %2 ==0)

  if ( P > maxP) {
    maxP = P;
  }
  if ( P < minP) {
    minP = P;
  }
                             //counter += 1;
                             // }  // end if counter % 60


//reset monthly rain to 0 on first day of month at midnight
  if (day()==1) {
    if (hour()==0){
       if (minute()==1){
          if (second()==0){
    mrainfall=0;
  }}}}


  // reset min and max values at 12 midnight
    if (hour() + minute() + second() == 0) {
    ymaxT = maxT;
    ymaxTTime = maxTTime;
    yrainfall = rainfall;

    yminT = (minT);
    yminTTime = minTTime;
    maxWTime=" ";

    maxws = 0;
    maxT = -100;
    maxP = 0;
    minT = 100;
    minP = 40;
    rainfall=0;
    delay(1000);
  }   // end if midnight reset

  contact();  //check for switch closing and calculate wind speed
  rain();     // check for bucket dump and calculate rainfall
  writeclient();  // write the web page

}  //end main loop



void writeclient() { //writeclient prints and manipulates global variables  
  // Check if a client has connected

  WiFiClient client = server.available();

  // Prepare the response

if (!client) {
    return;
    }
 
  client.flush();

  // Send the response to the client
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: text/html\r\n\r\n");  // essential to have 2 blank lines following
  client.print("<!DOCTYPE HTML\r\n");
  client.println("<html>\r\n");
  client.println("<head>\r\n");
  client.println("<title>Piney Weather</title>");
  client.println("<meta http-equiv=\"refresh\" content=\"30\" />");
  client.println("<meta encoding=\"utf-8\">");
 
  client.println("<style>");
  client.println("ul  {text-indent: 5px}");
  client.println(".temp  {color:red; list-style-type: none; font-size: large}");
  client.println(".rain  {color:blue; list-style-type: none; font-size: large }");
  client.println(".wind  {color:purple; list-style-type: none; font-size: large }");
  client.println(".baro  {color:navy; list-style-type: none; font-size: large }");
  client.println("</style>");
  
  
  
  client.println("</head>\r\n");

  client.println("<body>\r\n");      //get that font tag out of there
  
  client.println("<H1 align=\"left\"><font color=\"green\" font face=\"Comic Sans MS\">Piney Woods </font> Weather </H1>");
  client.println("<H3 align=\"left\">Trout Lake, WA </H3>");
  client.print ("<H4 align=\"left\">Date:  ");
  client.print(month());
  client.print("-");
  client.print(day());
  client.print("-");
  client.print(year() - 2000);
   client.print(" &nbsp; &nbsp; Time:  ");
  client.print(stringTime) ;
  client.println(" </H4>");
  client.println("<hr style= \"color: green; background-color: green; height: 2px\">");
   client.println("<ul>"); 
  client.print("<li class=\"temp\"><br>Temperature (F) = ");  
  client.print(T);
  client.println(" </li>");
  client.print("<li class=\"temp\"><br> Todays Min/Max Temp (F) = ");   
  client.print(minT);
  client.print(" at ");
  client.print(minTTime);
  client.print(" / ");
  client.print(maxT);
  client.print(" at ");
  client.print(maxTTime);
  client.println(" </li>");
  client.print("<li class=\"temp\"><br>Yesterdays Min/Max Temperatures ");  
  client.print(yminT);
  client.print(" at ");
  client.print(yminTTime);
  client.print(" / ");
  client.print(ymaxT);
  client.print(" at ");
  client.print(ymaxTTime);
  client.println(" </li>");

  client.print("<li class=\"wind\"><br>Wind Speed (mph) = ");
  client.print(ws);
  client.println(" </li>");


  client.print("<li class=\"wind\"><br>Max Wind Speed  =  "); 
  client.print(maxws);
  client.print(" at ");
  client.print(maxWTime);
  client.println(" </li>");
  
  client.print("<li class=\"baro\"><br>Barometer (in. Hg) = "); 
  client.print(P);
  client.println(" </li>");
  client.print("<li class=\"baro\"><br> Min/Max Pressure  =  ");
  client.print(minP);
  client.print(" / ");
  client.print(maxP);
  client.println(" </li>");
 
 client.print("<li class=\"rain\"><br> Rainfall (in) = ");
  client.print(rainfall);
   client.println(" </li>");
  client.print("<li class=\"rain\"><br> Yesterdays Rainfall =  ");
  client.print(yrainfall);
  client.println(" </li>");
client.print("<li class=\"rain\"><br> Monthly Rainfall =  ");
  client.print(mrainfall);
  client.println(" </li>");

client.println("</ul>");



//  client.println("<br>");
  client.println("Daily values reset at midnight \r\n");
//  client.println("<br>");

 // Historical Data
// client.println("<a href=\"https://67.237.227.70/shares/USB_Storage/PythonProgs/weather.csv\" target=\"_blank\">Historical Data</a>");
 
  
  
  
  //Weather Underground Sticker
  client.print("<p><h4><font face=\"Helvetica\" color=black><a href=\"http://www.wunderground.com/weatherstation/WXDailyHistory.asp?ID=KWATROUT3\" target=HI>Weather Underground</a></h4>");

   client.println("<br>");
  client.print("This session began ");
  client.println(upTime);
//  //Hit Counter
//  client.println("<br><br><br><br><br><br> ");
//  client.print("<a href=\"http://www.reliablecounter.com\" target=\"_blank\"><img ");
//  client.print("src=\"http://www.reliablecounter.com/count.php?page=67.237.227.70");
//  client.print("&digit=style/plain/3/&reloads=1\" alt=\"\" title=\"\" border=\"0\"></a><br /><a ");
//  client.print("href=\"http://www.curinglight.com\" target=\"_blank\" style=\"font-family: Geneva, ");
//  client.println("Arial; font-size: 6px; color: #330010; text-decoration: none;\">Polymerisationsleuchte</a>");


  client.println("</body>\r\n");
  client.println("</html>\r\n");

  // Serial.print("Wind Speed = ");
  //  Serial.println(ws);


}  //end writeclient



void contact()   
{
  // Serial.println("contact funct");
  // read the state of the anemometer switch:
  contactState = digitalRead(contactPin);

  if ( contactState == LOW) {
    contactState = HIGH; //Need to swap values for WeMos
  }
  else {
    contactState = LOW;
  }


  //Serial.println(contactState);
  if ( millis() - blip1 > 60000) { // set ws to zero when rotor stalls for more than 60 seconds
    ws = 0;
  }
  //  else {
  // check if the contact is closed.
  if (contactState == HIGH) {      //   *******************************

    //only calculate windspeed  if the contactState has gone from LOW to HIGH
    if (lastreading == LOW) {      //   *******************************

      // turn LED on on
      digitalWrite(BUILTIN_LED, 0);  //note LOW for WeMos



      (blip2) = millis();            // set blip2 to milliseconds (continuously running clock)


      et = ((blip2 - blip1) / 1000); // elapsed time is difference between current contact closure


      ws = ((360 / et) / 60);  //was 3600 for built in switch  360 assumes 10 revs per 1/60 mile

         Serial.print("Calculated ws= ");
        Serial.print(ws); 

      // Blynk.virtualWrite(V5, ws);   // Send Windspeed to Blynk

      //  Old end of 60th mile time (blip2) becomes new beginning time (blip1)
      blip1 = blip2;

      lastreading = HIGH;          // we wouldn't be in this loop if LO
      ws_old = ws;
      if (ws > maxws) {
        maxws = ws;
        maxWTime = stringTime;
      }

    }  //end if lastreading== LOW



  }  //end of if contactState == HIGH
  //  } // end of else for et < 60000
  else {
    //  Serial.println("contact 0");
    // contactState == Low
    // turn LED off:
    digitalWrite(BUILTIN_LED, 1);


    lastreading = LOW;
  }   // end of else for LOW contactState

  

}   // end of contact() function

void rain()                          //
{
 // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == LOW) {                                   //was high
      // if the current state is LOW then the button
      // went from HIGH to LOW:
      teeterCounter++;
//      Serial.println("on");
//      Serial.print("number of teeters:  ");
//      Serial.println(teeterCounter);
      rainfall=teeterCounter * .02;              //.02 inch per bucket dump
    } else {
      // if the current state is LOW then the button
      // wend from LOW to HIGH:
 //     Serial.println("off");
    }
   //  Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;

}  //end rain function

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
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
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
