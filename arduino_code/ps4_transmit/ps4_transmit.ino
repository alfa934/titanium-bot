#include <PS4Controller.h>

/* COMMENT OUT IF NOT NEEDED */
/* Print data to serial monitor */
// #define DEBUGGING_MODE
/* Send data to microcontroller */
#define SEND_DATA

// change ps4 address if needed
#define PS4_ADDR "3c:b6:b7:32:cd:f8"

char TransArray[43] = {'A', 'B', 'C'};

typedef struct {
  int8_t rX;
  int8_t rY;
  int8_t lX;
  int8_t lY;
  uint8_t r2;
  uint8_t l2;
  bool r1;
  bool l1;
  bool r3;
  bool l3;
  bool crs;
  bool sqr;
  bool tri;
  bool cir;
  bool up;
  bool down;
  bool right;
  bool left;
  bool share;
  bool option;

  bool ps;
  bool touchpad;
  uint8_t battery;

  int16_t gX, gY, gZ;
  int16_t aX, aY, aZ;
} Controller;



Controller input = {0};
int dummy = 0;

void setup() {
    #ifdef DEBUGGING_MODE
    Serial2.begin(115200);
    #endif

    #ifdef SEND_DATA
    Serial.begin(115200);
    #endif

    PS4.begin(PS4_ADDR);
}

void loop() {

    if(PS4.isConnected()) 
    {
      AssignControllerValue();
    }  
    else
    {
      memset(&input, 0, sizeof(Controller));
    }

    // Send data to STM32
    #ifdef SEND_DATA
    SendBytes();
    #endif
  
    #ifdef DEBUGGING_MODE
    PrintSerial2();
//    delay(50); // Delay for X second before sending the next data
    #endif
}

void PrintSerial2()
{
    Serial2.print(":");
    Serial2.print(dummy);
    Serial2.print(";");
    Serial2.print(input.lX);
    Serial2.print(";");
    Serial2.print(input.lY);
    Serial2.print(";");
    Serial2.print(input.rX);
    Serial2.print(";");
    Serial2.print(input.rY);
    Serial2.print(";");
    Serial2.print(input.r1);
    Serial2.print(";");
    Serial2.print(input.l1);
    Serial2.print(";");
    Serial2.print(input.r2);
    Serial2.print(";");
    Serial2.print(input.l2);
    Serial2.print(";");
    Serial2.print(input.sqr);
    Serial2.print(";");
    Serial2.print(input.crs);
    Serial2.print(";");
    Serial2.print(input.tri);
    Serial2.print(";");
    Serial2.print(input.cir);
    Serial2.print(";");
    Serial2.print(input.up);
    Serial2.print(";");
    Serial2.print(input.down);
    Serial2.println();
}



// add more buttons if needed
void AssignControllerValue()
{
    input.lX = PS4.LStickX();
    input.lY = PS4.LStickY();
    input.rX = PS4.RStickX();
    input.rY = PS4.RStickY();
    input.r1 = PS4.R1();
    input.l1 = PS4.L1();
    input.r2 = PS4.R2Value();
    input.l2 = PS4.L2Value();
    input.sqr = PS4.Square();
    input.crs = PS4.Cross();
    input.tri = PS4.Triangle();
    input.cir = PS4.Circle();
    input.up = PS4.Up();
    input.down = PS4.Down();
    input.left = PS4.Left();
    input.right = PS4.Right();
    input.r3 = PS4.R3();
    input.l3 = PS4.L3();
    input.share = PS4.Share();
    input.option = PS4.Options();

    input.ps = PS4.PSButton();
    input.touchpad = PS4.Touchpad();
    input.battery = PS4.Battery();

    input.gX = PS4.GyrX();
    input.gY = PS4.GyrY();
    input.gZ = PS4.GyrZ();
    
    input.aX = PS4.AccX();
    input.aY = PS4.AccY();
    input.aZ = PS4.AccZ();
}


void SendBytes()
{
    memcpy(TransArray + 3, &input.rX, 1);
    memcpy(TransArray + 4, &input.rY, 1);
    memcpy(TransArray + 5, &input.lX, 1);
    memcpy(TransArray + 6, &input.lY, 1);
    memcpy(TransArray + 7, &input.r2, 1);
    memcpy(TransArray + 8, &input.l2, 1);
    memcpy(TransArray + 9, &input.r1, 1);
    memcpy(TransArray + 10, &input.l1, 1);
    memcpy(TransArray + 11, &input.r3, 1);
    memcpy(TransArray + 12, &input.l3, 1);
    memcpy(TransArray + 13, &input.crs, 1);
    memcpy(TransArray + 14, &input.sqr, 1);
    memcpy(TransArray + 15, &input.tri, 1);
    memcpy(TransArray + 16, &input.cir, 1);
    memcpy(TransArray + 17, &input.up, 1);
    memcpy(TransArray + 18, &input.down, 1);
    memcpy(TransArray + 19, &input.right, 1);
    memcpy(TransArray + 20, &input.left, 1);
    memcpy(TransArray + 21, &input.share, 1);
    memcpy(TransArray + 22, &input.option, 1);
    
    memcpy(TransArray + 23, &input.ps, 1);
    memcpy(TransArray + 24, &input.touchpad, 1);
    memcpy(TransArray + 25, &input.battery, 1);

    memcpy(TransArray + 26, &input.gX, 2);
    memcpy(TransArray + 28, &input.gY, 2);
    memcpy(TransArray + 30, &input.gZ, 2);

    memcpy(TransArray + 32, &input.aX, 2);
    memcpy(TransArray + 34, &input.aY, 2);
    memcpy(TransArray + 36, &input.aZ, 2);
    
    for(int i = 0; i < sizeof(TransArray); i++)
    {
        Serial.write(TransArray[i]);
    }
}
