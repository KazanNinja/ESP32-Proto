TaskHandle_t Task1;
TaskHandle_t Task2;

// How many leds in your strip?
#define NUM_LEDS 2

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI

#include <ESP32-TWAI-CAN.hpp>

#include <Adafruit_NeoPixel.h>
#define DATA_PIN 36
#define NUMPIXELS 2

Adafruit_NeoPixel pixels(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

// Define the array of leds
//CRGB leds[NUM_LEDS];

// Default for ESP32
#define CAN_TX		11
#define CAN_RX		10

CanFrame rxFrame;

//Sets rpm, clt, gear integers for grabbing CAN data
int rpm;
int clt;
int gear;
int driverSwitch1;
int driverSwitch2;
int driverSwitch3;


//OBD TX frame setup
void sendObdFrame(uint8_t obdId) {
	CanFrame obdFrame = { 0 };
	obdFrame.identifier = 0x69; // Default OBD2 address;
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

void loop(){

}

void setup() {
    Serial.begin(115200);

    xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */

    xTaskCreatePinnedToCore(
      Task2code, /* Function to implement the task */
      "Task2", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &Task2,  /* Task handle. */
      1); /* Core where the task should run */

    //delay(2000); //Delay 

    //FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    //FastLED.setBrightness(255);  // Set global brightness to 50%
    //pinMode(35, INPUT_PULLUP);
    //pinMode(14, OUTPUT);
    
    //delay(2000);  // If something ever goes wrong this delay will allow upload.

    //pixels.begin();

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

    // or override everything in one command;
    // It is also safe to use .begin() without .end() as it calls it internally
    // if(ESP32Can.begin(ESP32Can.convertSpeed(1000), CAN_TX, CAN_RX, 10, 10)) {
    //     Serial.println("CAN bus started!");
    // } else {
    //     Serial.println("CAN bus failed!");
    // }

}

void Task1code(void *parameter) {
    //pixels.clear();
    while(true){
      //Serial.println("Running Task 1");
    static uint32_t lastStamp = 0;
    uint32_t currentStamp = millis();
    //int buttonState = digitalRead(35);

    if(currentStamp - lastStamp > 50) {   // sends OBD2 request every second
      lastStamp = currentStamp;
      //sendObdFrame(5); // For coolant temperature
    }
    
    // if(buttonState == 0){
    //   digitalWrite(14, HIGH);
    // }
    // else
    // {
    //   digitalWrite(14, LOW);
    // }

      if(ESP32Can.readFrame(rxFrame, 1000)) {
        
      //Comment out if too many frames
      //Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
        if(rxFrame.identifier == 0x640) {  
          //Serial.println("RECIEVED");
          // Standard OBD2 frame responce ID
          byte rpmLow = rxFrame.data[0];
          byte rpmHigh = rxFrame.data[1];
          rpm = (rpmLow << 8) + rpmHigh;
          //Serial.println(rpm);
        }

        if(rxFrame.identifier == 0x649){
          clt = rxFrame.data[0] - 40;
        }

        if(rxFrame.identifier == 0x64D){
          gear = rxFrame.data[6] & 0b00001111;
          
        }
        if(rxFrame.identifier == 0x64E){
          driverSwitch3 = rxFrame.data[3] & 0b01000000;
          Serial.println(driverSwitch3);
      }
  }

  // switch (gear) {
  //     case 0:  // your hand is on the sensor
  //       leds[0] = CRGB::White;
  //       FastLED.show();
  //       break;
  //     case 1:  // your hand is close to the sensor
  //       leds[0] = CRGB::Red;
  //       FastLED.show();
  //       break;
  //     case 2:  // your hand is a few inches from the sensor
  //       leds[0] = CRGB::Orange;
  //       FastLED.show();
  //       break;
  //     case 3:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Yellow;
  //       FastLED.show();
  //       break;
  //     case 4:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Green;
  //       FastLED.show();
  //       break;
  //     case 5:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Blue;
  //       FastLED.show();
  //       break;
  //     case 6:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Purple;
  //       FastLED.show();
  //       break;
  //     default:
  //       leds[0] = CRGB::Black;
  //       FastLED.show();
  //       break;
  // }

    // if(clt < 70){
    //   pixels.setPixelColor(0, pixels.Color(0,0,150));
    // }
    // else if(clt >= 70 && clt <= 100){
    //   pixels.setPixelColor(0, pixels.Color(0,150,0));
    // }
    // else if(clt > 100){
    //   pixels.setPixelColor(0, pixels.Color(150,0,0));
    // }
    // else{
    //   pixels.setPixelColor(0, pixels.Color(0,0,0));
    // }

    //pixels.show();
    //delay(10);


    //FastLED.show();

    // Serial.print("RPM: ");
    // Serial.println(rpm);

    // Serial.print("CLT: ");
    // Serial.println(clt);

    // Serial.print("GEAR: ");
    // Serial.println(gear);

    //FastLED.show();
    
    // Small delay to control the overall speed of the animation
    //delay(0.001);

  }
}
    

void Task2code(void *parameter2) {

  while(true){
    pixels.clear();
    //Serial.println("Task 2 Running");

  //   static uint32_t lastStamp = 0;
  //   uint32_t currentStamp = millis();
  //   //int buttonState = digitalRead(35);

  //   if(currentStamp - lastStamp > 50) {   // sends OBD2 request every second
  //     lastStamp = currentStamp;
  //     //sendObdFrame(5); // For coolant temperature
  //   }
    
  //   // if(buttonState == 0){
  //   //   digitalWrite(14, HIGH);
  //   // }
  //   // else
  //   // {
  //   //   digitalWrite(14, LOW);
  //   // }

  //     if(ESP32Can.readFrame(rxFrame, 1000)) {
        
  //     //Comment out if too many frames
  //     //Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
  //       if(rxFrame.identifier == 0x640) {  
  //         //Serial.println("RECIEVED");
  //         // Standard OBD2 frame responce ID
  //         byte rpmLow = rxFrame.data[0];
  //         byte rpmHigh = rxFrame.data[1];
  //         rpm = (rpmLow << 8) + rpmHigh;
            
  //       }

  //       if(rxFrame.identifier == 0x649){
  //         clt = rxFrame.data[0] - 40;
  //       }

  //       if(rxFrame.identifier == 0x64D){
  //         gear = rxFrame.data[6] & 0b00001111;
          
  //       }
  //       if(rxFrame.identifier == 0x64E){
  //         driverSwitch3 = rxFrame.data[3] & 0b01000000;
  //         Serial.println(driverSwitch3);
  //     }
  // }

  // switch (gear) {
  //     case 0:  // your hand is on the sensor
  //       leds[0] = CRGB::White;
  //       FastLED.show();
  //       break;
  //     case 1:  // your hand is close to the sensor
  //       leds[0] = CRGB::Red;
  //       FastLED.show();
  //       break;
  //     case 2:  // your hand is a few inches from the sensor
  //       leds[0] = CRGB::Orange;
  //       FastLED.show();
  //       break;
  //     case 3:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Yellow;
  //       FastLED.show();
  //       break;
  //     case 4:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Green;
  //       FastLED.show();
  //       break;
  //     case 5:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Blue;
  //       FastLED.show();
  //       break;
  //     case 6:  // your hand is nowhere near the sensor
  //       leds[0] = CRGB::Purple;
  //       FastLED.show();
  //       break;
  //     default:
  //       leds[0] = CRGB::Black;
  //       FastLED.show();
  //       break;
  // }
    Serial.print("TASK 2: ");
    Serial.println(clt);

    if(clt < 70){
      pixels.setPixelColor(0, pixels.Color(0,0,150));
    }
    else if(clt >= 70 && clt <= 100){
      pixels.setPixelColor(0, pixels.Color(0,150,0));
    }
    else if(clt > 100){
      pixels.setPixelColor(0, pixels.Color(150,0,0));
    }
    else{
      pixels.setPixelColor(0, pixels.Color(0,0,0));
    }

    if(driverSwitch3 == 64){
      pixels.setPixelColor(1, pixels.Color(150,150,150));
    }

    pixels.show();
    delay(1);

  }

  //Test

    //delay(10);


    //FastLED.show();

    // Serial.print("RPM: ");
    // Serial.println(rpm);

    // Serial.print("CLT: ");
    // Serial.println(clt);

    // Serial.print("GEAR: ");
    // Serial.println(gear);

    //FastLED.show();
    
    // Small delay to control the overall speed of the animation
    //delay(0.001);

}
