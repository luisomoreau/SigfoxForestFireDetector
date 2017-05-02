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

int sensorPin = 0;    // Attach the IR led to pin A1
int ledPin = 5;      // select the pin for the LED
int pinDHT11 = 2;
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

  // Display module informations
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
}

void loop()
{
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
      String msg = "1";
      sendString(msg);
      alarm = 0;
    } 
    delay(1000);
  }
  
  delay(100);
  digitalWrite(ledPin, LOW);
  LowPower.sleep();
  
}

void sendString(String str) {
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);

  SigFox.beginPacket();
  SigFox.print(str);

  int ret = SigFox.endPacket();  // send buffer to SIGFOX network
  if (ret > 0) {
    Serial.println("No transmission");
  } else {
    Serial.println("Transmission ok");
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
