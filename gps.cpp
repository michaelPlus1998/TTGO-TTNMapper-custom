#include "gps.h"

HardwareSerial GPSSerial(1);

void gps::init()
{
  GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  GPSSerial.setTimeout(2);
}

void gps::encode()
{       
    int data;
    int previousMillis = millis();

    while((previousMillis + 1000) > millis())
    {
        while (GPSSerial.available() )
        {
            char data = GPSSerial.read();
            tGps.encode(data);
            //Serial.print(data);
        }
    }
     //Serial.println("");
}

void gps::buildPacket(uint8_t payload[6])
{
  //try to do byte value...
  uint8_t txBuffer[6];
  
  LatitudeBinary = ((tGps.location.lat() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((tGps.location.lng() + 180) / 360.0) * 16777215;

  payload[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  payload[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  payload[2] = LatitudeBinary & 0xFF;

  payload[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  payload[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  payload[5] = LongitudeBinary & 0xFF;

  Serial.println(LatitudeBinary);
  Serial.println(LongitudeBinary);
  

  //Serial.println(latitude);
  //Serial.println(longitude);
  
  //payload[0] = latitude;
  //payload[1] = longitude;

  //Serial.println(tGps.location.lat(), 6);
  //Serial.println(tGps.location.lng(), 6);

  //Serial.print(payload[0]);
  //Serial.println(payload[1]);
  
}

bool gps::checkGpsFix()
{
  encode();
  if (tGps.location.isValid() && 
      tGps.location.age() < 2000 &&
      tGps.hdop.isValid() &&
      tGps.hdop.value() <= 300 &&
      tGps.hdop.age() < 2000 &&
      tGps.altitude.isValid() && 
      tGps.altitude.age() < 2000 )
  {
    Serial.println("Valid gps Fix.");
    return true;
  }
  else
  {
     Serial.println("No gps Fix.");
    // sprintf(t, "location valid: %i" , tGps.location.isValid());
    // Serial.println(t);
    // sprintf(t, "location age: %i" , tGps.location.age());
    // Serial.println(t);
    // sprintf(t, "hdop valid: %i" , tGps.hdop.isValid());
    // Serial.println(t);
    // sprintf(t, "hdop age: %i" , tGps.hdop.age());
    // Serial.println(t);
    // sprintf(t, "hdop: %i" , tGps.hdop.value());
    // Serial.println(t);
    // sprintf(t, "altitude valid: %i" , tGps.altitude.isValid());
    // Serial.println(t);
    // sprintf(t, "altitude age: %i" , tGps.altitude.age());
    // Serial.println(t);

    return false;
  }
}
