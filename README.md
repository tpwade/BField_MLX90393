# MLX90393 Sketch and connection details #

## references:
[https://www.melexis.com/en/product/MLX90393/Triaxis-Micropower-Magnetometer](https://www.melexis.com/en/product/MLX90393/Triaxis-Micropower-Magnetometer)  
[https://learn.adafruit.com/mlx90393-wide-range-3-axis-magnetometer](https://learn.adafruit.com/mlx90393-wide-range-3-axis-magnetometer)  
[https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button](https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button)

## Usage and Notes

This is a sketch for connecting and using an MLX90393 magnetometer to record gradient fields in and around MRI Gradient Coils.  The particular sensor I'm using comes from adafruit: 
[adafruit mlx90393](https://learn.adafruit.com/mlx90393-wide-range-3-axis-magnetometer).  Specifically the "Stemma QT" version.  I'm connecting it to a 5V Arduino Uno, and the MLX90393 is a 3.3V chip, but the board contains logic level shifters to handle the conversion.  It wasn't entirely clear whether the Stemma connectors went through the level shfters, so I wired directly to the labelled VIN, GND, SCL and SDA pins, which do.

There are two ways to trigger a reading.  Either enter "r" followed by "Enter" in the serial monitor, or press the button.  The button triggering is based off of [button example](https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button) There ought to be enough delays built in to the code to effectively de-bounce the button, but I didn't find it 100% reliable for taking readings.  Maybe I've just got a low quality button.

## Some other points:

The silkscreen on the adafruit board indicates the sensor coordinates, but is wrong. At least in the model I have, the directions for X and Y vectors should be changed. Refer to either the MLX90393 chip datasheet, or for reference, sparkfun has a similar board, but their silkscreen is correct. [https://www.sparkfun.com/products/retired/14160](https://www.sparkfun.com/products/retired/14160) 

## Code Install Notes
The code I have here is an evolution on the code I used to use for the Xtrinsic sensors board with the MAG3110 sensor.  In this case, I used the adafruit mlx90393 library unchanged: [library](https://github.com/adafruit/Adafruit_MLX90393_Library), commit 0115f42....  It should be installed in the arduino libraries directory (wherever that is on your system).  In may case, Ubuntu 20.04, Arduino 1.8.14, the libraries directory is ~/Arduino/libraries/Adafruit_MLX90393/. I'll provide a copy of the library with the code. 

The code itself needs to be in a parent directory that matches it's name, i.e. MLX90393/MLX90393.ino.

## Code Notes:


Set the analog gain and resolution (which 16 of the 19 ADC bits to output).
```
sensor.setGain(MLX90393_GAIN_5X);
sensor.setResolution(MLX90393_X, MLX90393_RES_16);
sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
```

Set the values that affect noise, ADC oversampling ration (OSR) and digital filter.
```
sensor.setOversampling(MLX90393_OSR_3);
sensor.setFilter(MLX90393_FILTER_6);
```
