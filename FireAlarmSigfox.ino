/*
 * Author: Louis Moreau
 * Date: 2nd May 2017
 * Description: Arduino sktech for the MKRFOX1200 to detect flame using IR sensor.
 * This code will wake the arduino up and read the temperature if a flame is detected
 * and then send this temparature using Sigfox network.
 * Moreover, one message is sent at a regular interval.
 * This code is in the Public Domain. Feel free to reuse it as you want.
 */

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <SimpleDHT.h>

#define MINUTESINMILLISECONDS 60 * 1000
#define DEBUG 0

int sensorPin = 1;    // Attach the IR led to pin D0
int ledPin = 3;      // select the pin for the LED
int pinDHT11 = 5;
int sensorValue = 0;  // variable to store the value coming from the sensor
int sleepTime = 60; // In minutes
int downlinkFrequency = 6; //Meaning requesting a downlink message every X uplinks
int countUplinks = 0;
int voltage = 0;
byte temperature = 0;
byte humidity = 0;

volatile int alarmFlag = 0;
volatile int aliveFlag = 1;

SimpleDHT11 dht11;



void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  delay(5000);

  if(DEBUG){
    Serial.begin(9600);
    while (!Serial) {};
  }
  

  if (!SigFox.begin()) {
    Serial.println("Shield error or not present!");
    reboot();
  }

  //Get Sigfox version, ID and PAC.
  String version = SigFox.SigVersion();
  String ID = SigFox.ID();
  String PAC = SigFox.PAC();

  // Display module information
  if(DEBUG){
    Serial.println("SigFox FW version " + version);
    Serial.println("ID  = " + ID);
    Serial.println("PAC = " + PAC); 
    Serial.println("");
    Serial.print("Module temperature: ");
    Serial.println(SigFox.internalTemperature());
  }
  


  delay(100);

  // Send the module to the deepest sleep
  SigFox.end();

  // attach pin 0 and 1 to a switch and enable the interrupt on voltage falling event
  pinMode(0, INPUT);
  LowPower.attachInterruptWakeup(0, alarmEvent, RISING);
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, keepAlive, CHANGE);
  //LowPower.sleep();
}

void loop()
{
  uint8_t msg[3];
  // read the value from the sensor:
  sensorValue = digitalRead(sensorPin);
  voltage = analogRead(ADC_BATTERY);
  voltage = map(voltage, 0, 1023, 0, 100);

  if(DEBUG){
    Serial.print("sensorValue :");
    Serial.println(sensorValue);
    Serial.print("Voltage :");
    Serial.println(voltage);
  }

  if(alarmFlag || aliveFlag){
    
    digitalWrite(ledPin,HIGH);
    if (dht11.read(pinDHT11, &temperature, &humidity, NULL)) {
      Serial.print("Read DHT11 failed.");
    }else{
      if(DEBUG){
        Serial.println();
        Serial.print("Sample OK: ");
        Serial.print((int)temperature); Serial.print(" ºC, ");
        Serial.print((int)humidity); Serial.println(" %");
      }
      
      //String msg = "1";
      msg[0] = uint8_t(temperature);
      msg[1] = uint8_t(humidity);
      msg[2] = uint8_t(voltage);
      msg[3] = uint8_t(alarmFlag);
      
      sendMsg(msg, 4);
    }
    
    //delay(1000);
  }
  
  
  delay(100);
  digitalWrite(ledPin, LOW);
  if(DEBUG){
    Serial.print("Back to sleep: ");
  }
  
  
  aliveFlag = 0;
  alarmFlag = 0;
  LowPower.sleep(sleepTime*MINUTESINMILLISECONDS);

}

void sendMsg(uint8_t msg[], int size) {
  uint8_t dlPayload[8];
  char output;
  bool askDL = false;
  
  //change the downlink request flag
  countUplinks++;
  if(DEBUG){
    Serial.print("Counter: ");
    Serial.println(countUplinks);
  }
  
  if(countUplinks>=downlinkFrequency){
    countUplinks = 0;
    askDL = true;
  }

  //Show payload in console
  int i=0;
  if(DEBUG){
    for(i=0;i<size;i++){
      Serial.println(msg[i]);
    }
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
  

  int ret = SigFox.endPacket(askDL);  // send buffer to SIGFOX network
  
  if(DEBUG){
    if (ret > 0) {
      Serial.println("No transmission");
    } else {
      Serial.println("Transmission ok");
    }
    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));
  }
  

  if(askDL){
    askDL = false;
    int j=0;
    if (SigFox.parsePacket()) {
      
      if(DEBUG){Serial.println("Response from server:");}
      while (SigFox.available()) {
        output = SigFox.read();
        dlPayload[j] = output;
        j++;
        if(DEBUG){
          Serial.print("0x");
          Serial.println(output, HEX);
        }  
        
      }
      changeConfiguration(dlPayload);
    } else {
      if(DEBUG){
        Serial.println("Could not get any response from the server");
        Serial.println("Check the SigFox coverage in your area");
        Serial.println("If you are indoor, check the 20dB coverage or move near a window");
      }
    }
    if(DEBUG){
      Serial.print("downlink payload: ");
      int k=0;
      for(k=0;k<8;k++){
        Serial.print(dlPayload[k], HEX);
      }
      Serial.println();
    }
  }

  SigFox.end();
}

void changeConfiguration(uint8_t dlPayload[]) {
  if(dlPayload[0]!=0){
    sleepTime = dlPayload[0]; // In minutes
    if(DEBUG){
      Serial.print("Sleep time configuration changed: ");
      Serial.print(sleepTime);
      Serial.println(" minutes.");
    }
  }
  
  if(dlPayload[1]!=0){
    downlinkFrequency = dlPayload[1]; //Meaning requesting a downlink message every X uplinks
    if(DEBUG){
      Serial.print("Downlink frequency configuration changed: ");
      Serial.println(downlinkFrequency);
    }
  }
  
} 


//Alarm event, triggered when sensor is rising
void alarmEvent() {
  alarmFlag = 1;
}

//Keep Alive event, triggered when the device wakes up from timer
void keepAlive(){
  aliveFlag = 1;
}

void reboot() {
  NVIC_SystemReset();
  while (1);
}
