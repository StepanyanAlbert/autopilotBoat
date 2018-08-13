
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h> 


//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2


SoftwareSerial mySerial(3, 2);

HMC5883L_Simple Compass;

const float destLong= 45.273127 ;
const float destLat=40.459863;
static int r = 6371;
const float Pi = 3.14159;
uint32_t start_time=millis();
int servoPin=9;
Adafruit_GPS GPS(&mySerial);
Servo Servo1; 

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


void setup()  
{

  Serial.begin(115200);
    Wire.begin();
 
    Compass.SetDeclination(+6, 14, 'E');  
    Compass.SetSamplingMode(COMPASS_SINGLE);
    Compass.SetScale(COMPASS_SCALE_088);
    Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
    
    //pin to see if gps fix is 1 
   pinMode(LED_BUILTIN, OUTPUT);
   
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
   GPS.begin(9600);
     Servo1.attach(servoPin);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
   useInterrupt(true);

  delay(1000);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void loop()                     
{

    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
//  Servo1.write(90); 
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

//      if (GPS.fix) {
        float targetHeading=GetHeading(40.205477,44.498136,destLat,destLong);
        float currentHeading = Compass.GetHeadingDegrees();
        float turnAngle;
        
        float differencceInAngles=targetHeading-currentHeading;        
        Serial.print("target heading");Serial.println(targetHeading);
        Serial.print("current heading");Serial.println(currentHeading);
        if( differencceInAngles>5 || differencceInAngles <0 ){
            if(differencceInAngles >=180){
               float correction;
                if(currentHeading-targetHeading <0){
                    correction=360+(currentHeading-targetHeading);
                  } else{
                    correction=360-(currentHeading-targetHeading);  
                  }
                  Serial.print("correction ");Serial.println(correction);
                  float targetAngle=map(correction, -180, 180, -60, 60);
                   turnAngle=targetAngle+90;
            }else{
               float correction;
               if(differencceInAngles <=-180){
                 correction=360-abs(differencceInAngles);
              }else{
                 correction=differencceInAngles;
                }
              float targetAngle=map(correctio n, -180, 180, -60, 60);
                Serial.print("correction ");Serial.println(correction);
              turnAngle=targetAngle+90;
              }
              Servo1.write(turnAngle); 
              delay(3000);
              //Servo1.write(90); 
         }
        Serial.print("turn angle ");Serial.print(turnAngle);
        digitalWrite(LED_BUILTIN, HIGH); 
        delay(2000);  
        digitalWrite(LED_BUILTIN, LOW);
//      }else{
//          digitalWrite(LED_BUILTIN, LOW);
//            Serial.println("No gps"); delay(1000);

//        }
//          Serial.print("Fix: "); 
//          Serial.print((int)GPS.fix);
//          Serial.print(" quality: ");
//          Serial.println((int)GPS.fixquality); 
          
           delay(500); 
//     
  }



static float ConvertToRadians(float angle)
{
  return (Pi / 180) * angle;
}
static float haversin(float input)
{
  return (sin(input / 2)) * (sin(input / 2));
}
static float ConvertToDegrees(float rad)
{
  float degrees = rad / (Pi / 180);
  if (degrees <= -180) {
    return degrees + 360;
  }
  else {
    return degrees;
  }
}
static float ConvertToStaticDegrees(float rad)
{
  float degrees = rad / (Pi / 180);
  if (degrees <= 0) {
    return degrees + 360;
  }
  else {
    return degrees;
  }
}
static float GetHeading(double sourceLat, double sourceLong, float destLat, float destLong)
{
  double lat1 = ConvertToRadians(sourceLat);
  double lat2 = ConvertToRadians(destLat);
  double long1 = ConvertToRadians(sourceLong);
  double long2 = ConvertToRadians(destLong);
  double dLon = long2 - long1;
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  return ConvertToStaticDegrees(atan2(y, x));
}
static float GetDistanceInM(double sourceLat, double sourceLong, float destLat, float destLong)
{
  //Current Latitude, Current Longitude
  //Projected Latitude, Projected Longitude
  double  lat1 = ConvertToRadians(sourceLat);
  double  lat2 = ConvertToRadians(destLat);
  double  long1 = ConvertToRadians(sourceLong);
  double  long2 = ConvertToRadians(destLong);
  double dLat = lat2 - lat1;
  double dLon = long2 - long1;
  double a = haversin(dLat) + cos(lat1) * cos(lat2) * haversin(dLon);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return r * c * 1000; // Distance in M
}
