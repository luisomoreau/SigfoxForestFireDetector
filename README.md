# Sigfox Flame Detector

## Introduction

How to prevent forest from burning? A small Sigfox signal can prevent this natural disaster. In this project, we will use an Arduino MKRFOX1200, a flame detector, a temperature and humidity sensors and we will send the data the TheThings.io platform.

I chose to put all the integrated electronic inside a small bird house, feel free to adapt the support!

Here it is what the first prototype looked like.

If you want to go further, you can add a solar panel with a charging circuit to charge a small battery, this part will not be covered by this tutorial. Feel free to submit a pull request on the [Github page](https://github.com/luisomoreau/SigfoxFlameDetector)

![picture1](img/picture1.jpg)

![picture2](img/picture2.jpg)

## What you need

* One [Arduino MKRFOX1200](https://www.arduino.cc/en/Main.ArduinoBoardMKRFox1200)
* One [flame detector](https://www.amazon.fr/Waveshare-Flame-Sensor-Detection-Raspberry/dp/B00NL5JEPS) or alternatively an IR led between 980nm and 1100 nm.
* A DHT11 temperature and humidity sensor.

## How to detect a fire using infrared sensors

I will try to be as much understandable as I can in this part. However, it is all about physics and I admit, I had some trouble to understand all the principles at the beginning.
I won't bother you the formulas but you can easily find them on Wikipedia.
In our case, a fire forest will mostly be a carbon fire.

Let's start with the Wien's law:


![Wien's law](https://upload.wikimedia.org/wikipedia/commons/a/a2/Wiens_law.svg)
Black body radiation as a function of wavelength for various absolute temperatures. Each curve is seen to peak at a somewhat different wavelength; Wien's law describes the shift of that peak in terms of temperature.

Now look at this graph with the temperature we are interested in.

![spectrum](img/spectrum.jpg)

The wavelengths corresponding to the peak emissions of the Sun (5777 K), the Earth's surface (300 K) and forest fires (600 to 1000 K).

The dashed line represents the 3.9 μm band and you see from Wien's law that 3.9 μm is the wavelength at which a blackbody at 743 K has its peak. This temperature corresponds to the temperature of a fire!

However, it is hard to get a 3.9 μm IR led. So we will use an [IR LED](https://www.amazon.fr/Waveshare-Flame-Sensor-Detection-Raspberry/dp/B00NL5JEPS) between 980nm and 1100 nm in this tutorial which has the following properties:
* The operating voltage is from 3.3 – 5V.
* It gives us both analog and digital output.
* It has a led indicator, which indicates that whether the flame is detected or not.
* The threshold value can be changes by rotating the top of potentiometer.
* Flame detection distance, lighter flame test can be triggered within 0.8m, if the intensity of flame is high, the detection distance will be increased.
* The detection angle of the flame sensor module is about 60 degrees.


Additionally, to avoid false positive alerts, some systems use two or three different IR (or UV) sensors.


## Hardware wiring

![sketch](img/sketch_picture.png)

## Arduino

```
/*
 * Author: Louis Moreau
 * Date: 2nd May 2017
 * Description: Arduino sktech for the MKRFOX1200 to detect flame using IR sensor.
 * This code will wake the arduino up and read the temperature if a flame is detected
 * and then send this temparature using Sigfox network.
 * This code is in the Public Domain. Feel free to reuse it as you want.
 */

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <SimpleDHT.h>

int sensorPin = 0;    // Attach the IR led to pin D0
int ledPin = 2;      // select the pin for the LED
int pinDHT11 = 5;
int sensorValue = 0;  // variable to store the value coming from the sensor
float voltage = 0;
byte temperature = 0;
byte humidity = 0;

volatile int alarm = 0;

SimpleDHT11 dht11;



void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {};

  if (!SigFox.begin()) {
    Serial.println("Shield error or not present!");
    reboot();
  }

  String version = SigFox.SigVersion();
  String ID = SigFox.ID();
  String PAC = SigFox.PAC();

  // Display module information
  Serial.println("SigFox FW version " + version);
  Serial.println("ID  = " + ID);
  Serial.println("PAC = " + PAC);

  Serial.println("");

  Serial.print("Module temperature: ");
  Serial.println(SigFox.internalTemperature());

  delay(100);

  // Send the module to the deepest sleep
  SigFox.end();

  // attach pin 0 and 1 to a switch and enable the interrupt on voltage falling event
  pinMode(0, INPUT);
  LowPower.attachInterruptWakeup(0, alarmEvent, RISING);
  LowPower.sleep();
}

void loop()
{
  uint8_t msg[2];
  // read the value from the sensor:
  sensorValue = digitalRead(sensorPin);

  Serial.print("sensorValue :");
  Serial.println(sensorValue);


//  Serial.print("Voltage :");
//  Serial.println(voltage);

  if(alarm){

    digitalWrite(ledPin,HIGH);
    if (dht11.read(pinDHT11, &temperature, &humidity, NULL)) {
      Serial.print("Read DHT11 failed.");
    }else{
      Serial.println();
      Serial.print("Sample OK: ");
      Serial.print((int)temperature); Serial.print(" *C, ");
      Serial.print((int)humidity); Serial.println(" %");
      //String msg = "1";
      msg[0] = uint8_t(temperature);
      msg[1] = uint8_t(humidity);
      msg[2] = uint8_t(0x01);
      sendMsg(msg, 3);
    }

    //delay(1000);
  }
  alarm = 0;

  delay(100);
  digitalWrite(ledPin, LOW);
  Serial.print("Back to sleep: ");
  LowPower.sleep();

}

void sendMsg(uint8_t msg[], int size) {
  int i=0;
  for(i=0;i<size;i++){
     Serial.println(msg[i]);
  }
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);

  SigFox.beginPacket();

  for(i=0;i<size;i++){
     SigFox.write(msg[i]);
  }


  int ret = SigFox.endPacket();  // send buffer to SIGFOX network
  if (ret > 0) {
    Serial.print("No transmission: ");
    Serial.println(ret);
  } else {
    Serial.print("Transmission ok: ");
    Serial.println(ret);
  }

  Serial.println(SigFox.status(SIGFOX));
  Serial.println(SigFox.status(ATMEL));
  SigFox.end();
}

void alarmEvent() {
  alarm = 1;
}

void reboot() {
  NVIC_SystemReset();
  while (1);
}

```
Let's try:

[![Youtube](https://img.youtube.com/vi/43xEpvAtNkc/0.jpg)](https://www.youtube.com/watch?v=43xEpvAtNkc)

## See your data in Sigfox Backend

First you need to activate your Arduino MKRFox 1200, for this:
- go to [Sigfox Backend](https://backend.sigfox.com/activate) and choose Arduino:

![Activate Screenshot](img/activate-1.png)

- Pick your country
- Add your device information
- Fill your account details

Then, log in [Sigfox Backend](https://backend.sigfox.com/) and go to [Device](https://backend.sigfox.com/device/list). You should be able to see your device:

![Device list screenshot](img/device-list-screenshot.png)

Click on its ID and go to message (on the left sidebar):

![Messages screenshot](img/messages-screenshots.png)

To see the decoding data, edit your device type information (on the information tab, the edit button is on the upper right side):

![Device type information](img/device-type-info.png)

And add the following parser:
- Payload parsing: Custom grammar
- Custom configuration: ```temperature::uint:8 humidity::uint:8 alert::bool:0```

![Edit device type info](img/edit-device-type.png)
