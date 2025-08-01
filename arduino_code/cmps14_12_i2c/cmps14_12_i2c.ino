#include <Wire.h>

#define CMPS12_ADDRESS 0x60
#define ANGLE_8  1

enum I2CState
{
  IDLE,
  REQUESTING,
  READING,
  PROCESSING
};

I2CState currentState = IDLE;
unsigned long lastRequestTime = 0;
const unsigned long I2C_TIMEOUT = 100;


unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;
float angle_deg_float = -1;
char tx_buffer[7] = "ABC";

void setup()
{
  Serial.begin(115200);
  Wire.begin();
}

void loop()
{
  unsigned long currentTime = millis();
  
  switch(currentState)
  {
    case IDLE:
      if (currentTime - lastRequestTime >= 5)
      {
        Wire.beginTransmission(CMPS12_ADDRESS);
        Wire.write(ANGLE_8);
        if (Wire.endTransmission() == 0)
        { 
          currentState = REQUESTING;
          lastRequestTime = currentTime;
        }
        else
        {
          lastRequestTime = currentTime + 100;
        }
      }
      break;
      
    case REQUESTING:
      Wire.requestFrom(CMPS12_ADDRESS, 5);
      currentState = READING;
      lastRequestTime = currentTime;
      break;
      
    case READING:
      if (Wire.available() >= 5)
      {
        angle8 = Wire.read();
        high_byte = Wire.read();
        low_byte = Wire.read();
        pitch = Wire.read();
        roll = Wire.read();
        currentState = PROCESSING;
      }
      else if (currentTime - lastRequestTime > I2C_TIMEOUT)
      {
        currentState = IDLE;
        lastRequestTime = currentTime + 100;
      }
      break;
      
    case PROCESSING:
      angle16 = (high_byte << 8) + low_byte;
      angle_deg_float = (float)angle16 / 10.0f;

      memcpy(tx_buffer + 3, &angle_deg_float, 4);

      for(int i = 0; i < sizeof(tx_buffer); i++)
      {
        Serial.write(tx_buffer[i]);
      }
      
      currentState = IDLE;
      break;
  }

}
