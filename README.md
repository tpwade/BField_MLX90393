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
The code I have here is an evolution on the code I used to use for the Xtrinsic sensors board with the MAG3110 sensor.  In this case, I used the adafruit mlx90393 library unchanged: [github library](https://github.com/adafruit/Adafruit_MLX90393_Library), commit 0115f42....  It should be installed in the arduino libraries directory (wherever that is on your system).  In may case, Ubuntu 20.04, Arduino 1.8.14, the libraries directory is ~/Arduino/libraries/Adafruit_MLX90393/. I'll provide a copy of the library with the code. 

The code itself needs to be in a parent directory that matches it's name, i.e. MLX90393_B/MLX90393_B.ino.

## Code Notes:

Set the analog gain and resolution (which 16 of the 19 ADC bits to output).  Gain_1X (0x7) and RES_16 (0x0) would be maximum gain, and lowest 16 bits of 19 bit ADC for highest precision measurement (0.15 uT/LSB on XY and 0.242 uT/LSB on Z).  The library gain defines are somewhat counter intuitive!!!

```
sensor.setGain(MLX90393_GAIN_1X);
sensor.setResolution(MLX90393_X, MLX90393_RES_16);
sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
```

Set the values that affect noise, ADC oversampling ratio (OSR) and digital filter. OSR_3 (0x3) and FILTER_7 (0x7) provide the least noise, but because of all the filtering, the lowest data rate, with a Tconv of 200.37 ms, or data rate of 4.5 samples/s.

```
sensor.setOversampling(MLX90393_OSR_3);
sensor.setFilter(MLX90393_FILTER_7);
```

The MAG3110 magnetometer has a status byte, that indicates when new data is available since the last read.  There doesn't seem to be anything equivalent on the MLX90393.

## Not investigated

The MLX90393 has some features that I have not tested/investigated, and have just left at defaults, or whatever was in the library.

It has an onboard temperature sensor which can be used for temperature correction. For now I'm just assuming that it does this correction automatically by default.

HALL_CONF controls the hall plate spinning. I'm not sure what the implications of this are.  I leave it unchanged at what I'm assuming is the default of 0xC, which has the maximum precision.

Resets might be necessary if the reading becomes stuck. There were some issues along these lines with the MAG3110, but so far the MLX90393 seems fine.

Offsets are needed for absolute field measurements, and require a calibration routine. Given that we are substracting the background field, I don't believe this is necessary, and I haven't implemented anything along these lines.

Burst vs Single Measurement mode vs ?. For now I'm assuming that each trigger reads a new measurement, but these modes should probably be investigated further.

