#include <Sgp4.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <RTCZero.h>
#include <AccelStepper.h>

#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>


// To be modified by user //
#define SECRET_SSID "Sevgi"                                     // Your Network name.
#define SECRET_PASS "1234"                                         // Your Network password.
#define DEBUG                                                          //Enable serial output.
const int timeZone = -7;                                               //Your Time zone.
char TLE[500];                                                         //Variable to store satellite TLEs.
char satnames[4][30] = {"RADARSAT-2", "NEOSSAT", "M3MSAT ", "SCISAT"}; // Names of satellites.
char satURL[4][30] = {"/satcat/tle.php?CATNR=32382", "/satcat/tle.php?CATNR=39089",
                      "/satcat/tle.php?CATNR=41605", "/satcat/tle.php?CATNR=27858"}; // URL of Celestrak TLEs for satellites (In same order as names).
char TLE1[4][70];
char TLE2[4][70];
float myLat = 52.12;
float myLong = -106.663;
float myAlt = 482; // Your latitude, longitude and altitude.
int numSats = 4;   // Number of satellites to track.

// Azimuth stepper pins //
#define AZmotorPin1 9  // IN1 on the ULN2003 driver
#define AZmotorPin2 10 // IN2 on the ULN2003 driver
#define AZmotorPin3 11 // IN3 on the ULN2003 driver
#define AZmotorPin4 12 // IN4 on the ULN2003 driver
// Elevation stepper pins //
#define ELmotorPin1 2
#define ELmotorPin2 3
#define ELmotorPin3 4
#define ELmotorPin4 5

float flat,flon; // create variable for latitude and longitude object
SoftwareSerial gpsSerial(6,7);//tx,rx
LiquidCrystal lcd(A0,A1,A2,A3,A4,A5);
TinyGPS gps; // create gps object


int satAZsteps;
int satELsteps;
int turns = 0;
float oneTurn = 4096;        // Number of steps per one rotation for stepper motor.
#define MotorInterfaceType 8 // Define the AccelStepper interface type; 4 wire motor in half step mode:
AccelStepper stepperAZ = AccelStepper(MotorInterfaceType, AZmotorPin1, AZmotorPin3, AZmotorPin2, AZmotorPin4);
AccelStepper stepperEL = AccelStepper(MotorInterfaceType, ELmotorPin1, ELmotorPin3, ELmotorPin2, ELmotorPin4);

Sgp4 sat;
RTCZero rtc;

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
int i;
int k;
int SAT;
int nextSat;
int AZstart;
long passEnd;
int satVIS;
char satname[] = " ";
int passStatus = 0;
char server[] = "104.168.149.178"; //Web address to get TLE (CELESTRAK)
int year;
int mon;
int day;
int hr;
int minute;
double sec;
int today;
long nextpassEpoch;
long upcomingPasses[4];
int status = WL_IDLE_STATUS;
unsigned long unixtime;
unsigned long testTime = 1593789900;
unsigned long timeNow = 0;

// Used for Network Time Protocol (NTP)
unsigned int localPort = 2390;        // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48;       // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];   //buffer to hold incoming and outgoing packets
WiFiUDP Udp;                          // A UDP instance to let us send and receive packets over UDP

// Initialize the Ethernet client library
WiFiClient client;

void setup()
{


//Initialize serial and wait for port to open:
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10);
  }
#endif



Serial.begin(9600); // connect serial
Serial.println("The GPS Received Signal:");
gpsSerial.begin(9600); // connect gps sensor
lcd.begin(16,2);
lcd.setCursor(0,0);
lcd.print(" GPS_Location");
lcd.setCursor(0,1);
lcd.print(" Shashwat__Raj");
delay(8000);
lcd.clear();



  // Setup stepper movements //
  stepperEL.setMaxSpeed(1000);
  stepperEL.setCurrentPosition(-227); // Elevation stepper starts at -227 steps (20 degrees above horizon).
  stepperEL.setAcceleration(100);
  stepperAZ.setMaxSpeed(1000);
  stepperAZ.setCurrentPosition(0); // Azimuth stepper starts at 0.
  stepperAZ.setAcceleration(100);

  // check for the presence of wifi shield:
  if (WiFi.status() == WL_NO_SHIELD)
  {
#ifdef DEBUG
    Serial.println("WiFi shield not present");
#endif
    while (true)
      ; // don't continue:
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
#ifdef DEBUG
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
#endif
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 5 seconds for connection:
    delay(5000);
  }
#ifdef DEBUG
  Serial.println("Connected to wifi");
#endif

  sat.site(myLat, myLong, myAlt); //set location latitude[°], longitude[°] and altitude[m]
  rtc.begin();

  // Get Unix time //
  while (timeNow == 0)
  {
    unixtime = readLinuxEpochUsingNTP();
    rtc.setEpoch(unixtime);
    today = rtc.getDay();
    timeNow = rtc.getEpoch();
  }
#ifdef DEBUG
  Serial.println("unixtime: " + String(unixtime));
#endif

  // Get TLEs //
  for (SAT = 0; SAT < numSats; SAT++)
  {
    getTLE(SAT);
    sat.init(satname, TLE1[SAT], TLE2[SAT]); //initialize satellite parameters
    sat.findsat(timeNow);
    upcomingPasses[SAT] = Predict(1);
  }
  nextSat = nextSatPass(upcomingPasses);
  sat.init(satname, TLE1[nextSat], TLE2[nextSat]);
  Predict(1);

// Print obtained TLE in serial. //
#ifdef DEBUG
  for (SAT = 0; SAT < numSats; SAT++)
  {
    Serial.println("TLE set #:" + String(SAT));
    for (i = 0; i < 70; i++)
    {
      Serial.print(TLE1[SAT][i]);
    }
    Serial.println();
    for (i = 0; i < 70; i++)
    {
      Serial.print(TLE2[SAT][i]);
    }
    Serial.println();
  }
  Serial.println("Next satellite: " + String(nextSat));
#endif
}


static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}




void loop()
{

  smartdelay(1000);

  //gps.f_get_position(&flat, &flon);



  
//while(gpsSerial.available()){ // check for gps data
//if(gps.encode(gpsSerial.read()))// encode gps data
//{
//gps.f_get_position(&lat,&lon); // get latitude and longitude

// display position
lcd.clear();
lcd.setCursor(1,0);
lcd.print("GPS Signal");

lcd.setCursor(1,0);
lcd.print("LAT:");
lcd.setCursor(5,0);
lcd.print(flat,7);
Serial.print(flat,7);
Serial.print(" ");
Serial.print(flon,7);
Serial.print(" ");
lcd.setCursor(0,1);
lcd.print(" LON:");
lcd.setCursor(5,1);
lcd.print(flon,7);
//}
//}

String latitude = String(flat,6);
String longitude = String(flon,6);
Serial.println(latitude+";"+longitude);
delay(1000);
stepperEL.runToNewPosition(-227); //Standby at 20 degrees above horizon
  delay(1000);
    stepperEL.runToNewPosition(0); //Standby at 20 degrees above horizon
}
