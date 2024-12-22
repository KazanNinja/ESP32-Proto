#include <ESP32-TWAI-CAN.hpp>
#include <Adafruit_NeoPixel.h>

//Setting Neopixel data output pin
//Setting number of Neopixels in chain
#define DATA_PIN 36
#define NUMPIXELS 2
Adafruit_NeoPixel pixels(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);

// CAN TX-RX Pins
#define CAN_TX		11
#define CAN_RX		10

//CAN Recieving frame object
CanFrame rxFrame;

//Defining Tasks
TaskHandle_t CAN_Task;
TaskHandle_t Neopixel_Task;

//Defines global rpm, clt, gear, etc integers for CAN data
int rpm;
int clt;
int gear;
int driverSwitch3;

//Value at which the "Cold" and "Hot" lights are turned on, in Celsius
//Value at which light starts flashing
int coolantCold = 70;
int coolantHot = 105;
int coolantFlash = 120;

//OBD TX frame setup
void sendObdFrame(uint8_t obdId) {
	CanFrame obdFrame = { 0 };
	obdFrame.identifier = 0x0F3; // Default OBD2 address;
	obdFrame.extd = 0;
	obdFrame.data_length_code = 8;
	obdFrame.data[0] = 2;
	obdFrame.data[1] = 1;
	obdFrame.data[2] = obdId;
	obdFrame.data[3] = 0xAA;    // Best to use 0xAA (0b10101010) instead of 0
	obdFrame.data[4] = 0xAA;    // CAN works better this way as it needs
	obdFrame.data[5] = 0xAA;    // to avoid bit-stuffing
	obdFrame.data[6] = 0xAA;
	obdFrame.data[7] = 0xAA;
  // Accepts both pointers and references 
  //ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}

void setup() {
  //Begin Serial comms
  Serial.begin(115200);

  //Set pinModes for onboard button and onboard LED
  pinMode(35, INPUT_PULLUP);
  pinMode(14, OUTPUT);

  //Setting up task for CAN bus stuffz
  xTaskCreatePinnedToCore(
    CAN_Task_Code, //Function to implement the task
    "CAN Task",    //Name of the task
    10000,         //Stack size in words
    NULL,          //Task input parameter
    0,             //Priority of the task
    &CAN_Task,     //Task handle
    0              //Core where the task should run
  );            

  //Settings up task for Neopixel lighting
  xTaskCreatePinnedToCore(
    Neopixel_Task_Code, //Function to implement the task
    "Neopixel Task",    //Name of the task
    10000,              //Stack size in words
    NULL,               //Task input parameter
    1,                  //Priority of the task
    &Neopixel_Task,     //Task handle
    1                   //Core where the task should run
  );                 

  //CAN setup
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
	ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

  // You can also just use .begin()..
  if(ESP32Can.begin()) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }
}

void loop(){
  //Do nothing :)
}

void CAN_Task_Code(void *parameter) {

  while(true){

    //Shitasses
    static uint32_t lastStamp = 0;
    uint32_t currentStamp = millis();

    //Onboard button
    int buttonState = digitalRead(35);

    //CAN TX-ing
    // if(currentStamp - lastStamp > 50) {   // sends OBD2 request every second
    //   lastStamp = currentStamp;
    //   sendObdFrame(5); // For coolant temperature
    // }
    
    //Onboard button LED driving, button pressed (0, pulled up) turn the LED on
    if(buttonState == 0){
      digitalWrite(14, HIGH);
    }
    else
    {
      digitalWrite(14, LOW);
    }

    //Checks if there are any frames to read
    if(ESP32Can.readFrame(rxFrame, 1000)) {

      //Engine Speed CAN Frame
      if(rxFrame.identifier == 0x640) {  
        byte rpmLow = rxFrame.data[0];
        byte rpmHigh = rxFrame.data[1];
        rpm = (rpmLow << 8) + rpmHigh;
      }
    
      //Goolant CAN Frame
      if(rxFrame.identifier == 0x649){
        clt = rxFrame.data[0] - 40;
      }

      //Gear position CAN Frame
      if(rxFrame.identifier == 0x64D){
        gear = rxFrame.data[6] & 0b00001111;
      }

      //Pit Switch CAN Frame
      if(rxFrame.identifier == 0x64E){
        driverSwitch3 = rxFrame.data[3] & 0b01000000;
      }
    }
  }
}
    
void Neopixel_Task_Code(void *parameter2) {

  while(true){

    //Clears any existing pixels 
    pixels.clear();

    //Switch statement for gear position light
    switch (gear) {
      case 0:  // Gear position 0
        pixels.setPixelColor(0, pixels.Color(255,0,0));
        break;
      case 1:  // Gear position 1
        pixels.setPixelColor(0, pixels.Color(0,255,0));
        break;
      case 2:  // Gear position 2
        pixels.setPixelColor(0, pixels.Color(0,0,255));
        break;
      case 3:  // Gear position 3
        pixels.setPixelColor(0, pixels.Color(0,255,255));
        break;
      case 4:  // Gear position 4
        pixels.setPixelColor(0, pixels.Color(255,255,0));
        break;
      case 5:  // Gear position 5
        pixels.setPixelColor(0, pixels.Color(255,0,255));
        break;
      case 6:  // Gear position 6
        pixels.setPixelColor(0, pixels.Color(255,255,255));
        break;
      default: // Gear position fallback, no lighting
        pixels.setPixelColor(0, pixels.Color(0,0,0));
        break;
    }

    //Coolant lighting Neopixel
    if(clt < coolantCold){
      pixels.setPixelColor(0, pixels.Color(0,0,255));
    }
    else if(clt >= coolantCold && clt <= coolantHot){
      pixels.setPixelColor(0, pixels.Color(0,255,0));
    }
    else if(clt > coolantHot){
      pixels.setPixelColor(0, pixels.Color(255,0,0));
    }
    else{
      pixels.setPixelColor(0, pixels.Color(0,0,0));
    }

    //Pit Switch Input
    if(driverSwitch3 == 64){
      pixels.setPixelColor(1, pixels.Color(255,255,255));
    }

    //Sets pixel output, 1ms delay
    pixels.show();
    delay(1);

  }
}
