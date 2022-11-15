#include "Adafruit_MLX90393_TPW.h"
#include <Adafruit_I2CDevice.h>

Adafruit_MLX90393 sensor = Adafruit_MLX90393();
//Adafruit_I2CDevice *i2c_dev = NULL;
#define MLX90393_CS 10

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 4;     // the number of the pushbutton pin
//const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int16_t data_pt;

void setup() {
  
//  char inChar;
  
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

    
//    Serial.println("Run CalLoop? (y/n)");
//    inChar = 'a';
//    while( (inChar!='y') || (inChar!='n') ) {
//        inChar = Serial.read();
//        if (inChar == 'y') {
//          Serial.println("While loop is running, rotate sensor slowly about each axis");
//          CalLoop();
//        }
//    }
  
    
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
        if (inChar == 'n') Serial.println("NEWLINE");
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

uint8_t CalLoop() {
    
    float x, y, z;
    uint16_t offset, xi,yi,zi,ti;
    uint8_t txbuf[1];
    uint8_t rxbuf[9] = {0};
    uint8_t interdelay = 0;
    uint8_t status, status_sm;
    
    /* Set analog gain and precision
     * GAIN_1X = 0x07
     * RES_16 = 0x00 (lowest 16 bits of 19bit ADC)
     * so Gain = 0x07, Res=0x00 is 0.150/0.242 uT/LBS
     * on XY/Z 
     * */
    sensor.setGain(MLX90393_GAIN_1X);
    sensor.setResolution(MLX90393_X, MLX90393_RES_16); 
    sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
    sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
    
    /* Set oversampling and digital filtering
     * OSR_2 = 0x02
     * FILTER_6 = 0x06
     * above result in a Tconv = 51.38 ms, but also still relatively low noise ~< 5 mGauss (0.5 uT)
     * */
    sensor.setOversampling(MLX90393_OSR_2);
    sensor.setFilter(MLX90393_FILTER_6);

    delay(10);
    
    
    Serial.println("Offset Values X, Y, Z");
    sensor.readRegister(0x04,&offset); // OFFSET_X
    Serial.print(offset); Serial.print(", ");
    sensor.readRegister(0x05,&offset); // OFFSET_Y
    Serial.print(offset); Serial.print(", ");
    sensor.readRegister(0x06,&offset); // OFFSET_Z
    Serial.print(offset); Serial.println();
    
    delay(1000);
    
    Serial.println("Raw Values: SM status, RM status, T, X, Y, Z");
    
    for (int i=0; i<1000; i++){

      delay(5);
    
        /* Start a single measurement. */
        txbuf[0] = MLX90393_REG_SM | MLX90393_AXIS_ALL | 0x01;
        //Serial.print("tx command: ");
        //Serial.println(txbuf[0]);
        if (sensor.i2c_dev) {
            if (!sensor.i2c_dev->write(txbuf, 1))
                return false;
            delay(interdelay);
            
            if (!sensor.i2c_dev->read(&status_sm, 1))
                return false;

            //Serial.print("Start Measurement Status: ");
            //Serial.print(status_sm); Serial.println();
        }
        
        delay(65);
        
        
        /* Read a single data sample. */
        txbuf[0] = MLX90393_REG_RM | MLX90393_AXIS_ALL | 0x01;
        //Serial.print("tx command:");
        //Serial.println(txbuf[0]);
        
        /*
        status = sensor.transceive(txbuf,1,rxbuf,6,0);
        Serial.println(status);
        */
        
        if (sensor.i2c_dev) {
            if (!sensor.i2c_dev->write(txbuf, 1))
                return false;
            delay(interdelay);
            
            if (!sensor.i2c_dev->read(rxbuf, 9))
                return false;
            //Serial.println(rxbuf[0]);
            //Serial.println(rxbuf[1]);
            //Serial.println(rxbuf[2]);
            //Serial.println(rxbuf[3]);
            //Serial.println(rxbuf[4]);
        }
        status = rxbuf[0];
        ti = (rxbuf[1] << 8) | rxbuf[2];
        xi = (rxbuf[3] << 8) | rxbuf[4];
        yi = (rxbuf[5] << 8) | rxbuf[6];
        zi = (rxbuf[7] << 8) | rxbuf[8];
        
        //Serial.println("Raw Values: i, SM status, RM status, T, X, Y, Z");
        Serial.print(status_sm); Serial.print(", ");
        Serial.print(i); Serial.print(", ");
        Serial.print(status); Serial.print(", ");
        Serial.print(ti); Serial.print(", ");
        Serial.print(xi); Serial.print(", ");
        Serial.print(yi); Serial.print(", ");
        Serial.print(zi); Serial.println();

    }
    return false;
    
    
    /* Start a single measurement. */
    txbuf[1] = {MLX90393_REG_SM | MLX90393_AXIS_ALL};
    Serial.println(txbuf[1]);
    if (sensor.i2c_dev) {
        if (!sensor.i2c_dev->write(txbuf, 1))
            return false;
        delay(interdelay);
        
        if (!sensor.i2c_dev->read(&status, 1))
            return false;

        //status = rxbuf[0];
        Serial.print("Start Measurement Status: ");
        Serial.print(status,BIN); Serial.println();
    }
    
    delay(200);
    
    /* Read a single data sample. */
    if (sensor.i2c_dev) {
        if (!sensor.i2c_dev->write(txbuf, 1))
            return false;
        delay(interdelay);
        
        if (!sensor.i2c_dev->read(rxbuf, 7))
            return false;
    }
    
    status = rxbuf[0];
    ti = (rxbuf[1] << 8) | rxbuf[2];
    xi = (rxbuf[3] << 8) | rxbuf[4];
    yi = (rxbuf[5] << 8) | rxbuf[6];
    zi = (rxbuf[7] << 8) | rxbuf[8];
    
    Serial.println("Raw Values: Status, T, X, Y, Z");
    Serial.print(status,BIN); Serial.print(", ");
    Serial.print(ti); Serial.print(", ");
    Serial.print(xi); Serial.print(", ");
    Serial.print(yi); Serial.print(", ");
    Serial.print(zi); Serial.println();
    
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

  delay(200);

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event;
  sensor.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("X: "); Serial.print(event.magnetic.x);
  Serial.print(" \tY: "); Serial.print(event.magnetic.y);
  Serial.print(" \tZ: "); Serial.print(event.magnetic.z);
  Serial.println(" uTesla ");

  delay(200);

}
