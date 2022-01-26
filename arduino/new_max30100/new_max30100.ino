/*

Download Libraries:
https://www.electroniclinic.com/arduino-libraries-download-and-projects-they-are-used-in-project-codes/

*/

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     3000

 PulseOximeter pox;
uint32_t tsLastReport = 0;
 
void onBeatDetected()
{
    ;
}
 
void setup()
{
    Serial.begin(9600);
    Serial.print("Initializing pulse oximeter..");
  

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
        digitalWrite(1,HIGH);
    }
     pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
 
    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);

  
}
 
void loop()
{

    // Make sure to call update as fast as possible
    pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {


  // to computer Serial Monitor
        Serial.print("BPM: ");
        Serial.print(pox.getHeartRate());
        //blue.println("\n");
        
        Serial.print("    SpO2: ");
        Serial.print(pox.getSpO2());
        Serial.print("%");
        Serial.println("\n");
        

        tsLastReport = millis();
        
    }

}
