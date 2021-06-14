#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 sensor = Adafruit_MLX90393();
#define MLX90393_CS 10

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 4;     // the number of the pushbutton pin
//const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int16_t data_pt;

void setup() {
  
  // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
      delay(10);
  }

  Serial.println("Starting Adafruit MLX90393 Demo");

  if (! sensor.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! sensor.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
  Serial.println("Found a MLX90393 sensor");

  sensor.setGain(MLX90393_GAIN_1X);
  // You can check the gain too
  Serial.print("Gain set to: ");
  switch (sensor.getGain()) {
    case MLX90393_GAIN_1X: Serial.println("1 x"); break;
    case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
    case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
    case MLX90393_GAIN_2X: Serial.println("2 x"); break;
    case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
    case MLX90393_GAIN_3X: Serial.println("3 x"); break;
    case MLX90393_GAIN_4X: Serial.println("4 x"); break;
    case MLX90393_GAIN_5X: Serial.println("5 x"); break;
  }

  // Set resolution, per axis
  sensor.setResolution(MLX90393_X, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  sensor.setFilter(MLX90393_FILTER_7);
}

void loop() {
    
    // put your main code here, to run repeatedly:
    float x, y, z;
    char inChar;
    
    //https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button

    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);
    
    inChar = 'a';
    while( (inChar!='r') && buttonState == LOW ) {
        inChar = Serial.read();
        buttonState = digitalRead(buttonPin);
    }
    
      // get X Y and Z data at once
    if (sensor.readData(&x, &y, &z)) {
        Serial.print(data_pt);
        Serial.print(",");
        Serial.print(x,2);
        Serial.print(",");
        Serial.print(y,2);
        Serial.print(",");
        Serial.println(z,2);
    } else {
        Serial.println("Unable to read XYZ data from the sensor.");
    }

    delay(100);
    
    //Wait for button to be released
    while( buttonState == HIGH ) {
        buttonState = digitalRead(buttonPin);
    }
    
    // for de-bounce
    delay(100);
    
    data_pt++;
    
}

void OLDloop() {

    
    
  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    //digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    //digitalWrite(ledPin, LOW);
  }
  
  // put your main code here, to run repeatedly:
  float x, y, z;

  // get X Y and Z data at once
  if (sensor.readData(&x, &y, &z)) {
      Serial.print("X: "); Serial.print(x, 4); Serial.println(" uT");
      Serial.print("Y: "); Serial.print(y, 4); Serial.println(" uT");
      Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }

  delay(500);

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event;
  sensor.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("X: "); Serial.print(event.magnetic.x);
  Serial.print(" \tY: "); Serial.print(event.magnetic.y);
  Serial.print(" \tZ: "); Serial.print(event.magnetic.z);
  Serial.println(" uTesla ");

  delay(500);

}
