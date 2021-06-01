#define BLYNK_PRINT Serial



#include <ESP8266WiFi.h>

#include <BlynkSimpleEsp8266.h>



// You should get Auth Token in the Blynk App.

// Go to the Project Settings (nut icon).

char auth[] = "cZEG1imfnTxazdpHURFM5_L2Z-RL8dzC";



// Your WiFi credentials.

// Set password to "" for open networks.

char ssid[] = "zeza12345";

char pass[] = "0816176188";

int peopleCount = 0;

int doorState = 0;

BlynkTimer timer;
int toggle =0;

BLYNK_WRITE(V5)
{
  int pinValue = param.asInt();
  if(pinValue == 1 && toggle == 0){
    if(peopleCount>0){ 
      peopleCount--;
      Serial.write('d');
      }
    toggle = 1;
    Blynk.virtualWrite(V3,peopleCount);
    }
   if (pinValue == 0 ){
    toggle = 0;
    }
}
void setup()

{

  // Debug console
  timer.setInterval(400L, sensorDataSend);
  Serial.begin(115200);

  Blynk.begin(auth, ssid, pass);
  
   
}
  

 
void sensorDataSend()
{
  if (Serial.available()) {
   char in = Serial.read();
   
   if(in == 'o'){
    doorState = 1;
    Blynk.virtualWrite(V2,doorState);
   }
   else if(in == 'c'){
    doorState = 0;
    Blynk.virtualWrite(V2,doorState);
   }
   else if(in == 'i'){
    peopleCount++;
    Blynk.virtualWrite(V3,peopleCount);
   }
  
   }
}


void loop(){

  Blynk.run();
  timer.run();
}
