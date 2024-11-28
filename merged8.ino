// MECH0020 Smart Irrigation Device //

//Temp & Humidity Sensor Library Import and Pin Definition
#include "dht11.h"
dht11 DHT;
#define DHT11_PIN 46 //Temperature & Humidity Sensor Pin 46

//Software Serial Communication Library Import and Setup (for communication with Wi-Fi module)
#include <SoftwareSerial.h>
SoftwareSerial espSerial(10, 11); //Software serial - Pin 10 RX, Pin 11 TX

//Watering Relay and Rain sensor Pin Definitions
const int relayPin = 3; 
const int rainPin = 2;

//Define Soil Moisture Sensor Measurement variable (integer)
int soilMoisture = 0;

//Manual Watering Button Function
volatile byte buttonReleased = false;
void buttonReleasedInterrupt() {
  buttonReleased = true;
}

//Define pulse counter variable for counting bucket tips of rain gauge
volatile int pulseCount = 0;

//Maximum and Minimum Temperature variables setup
float maxTemp = 0;
float minTemp = 100;

//Maximum and Minimum Humidity variables setup
float maxHumidity = 0;
float minHumidity = 100;

//Water Balance variable and 24 hour counter Setup
volatile float WB = 0;
unsigned long previousDayCounter = 0;
const long dayInterval = 86400000; //24 hours = (86400000 ms)

//Water Flow Sensor variables setup
volatile double water;
const int waterPin = 19;
volatile byte waterState = false;

//Pulse Counter Function (for rain gauge)
void countPulse() {
  unsigned long bucketTime = millis();
  static unsigned long lastBucketTime = 0;
  if (bucketTime - lastBucketTime > 300) { //Check if no other pulse within 300ms
    pulseCount++;
    Serial.print("Bucket has Tipped ");
    Serial.print(pulseCount);
    Serial.println(" times!");
    WB = WB + 0.3537; //Add 0.3537mm to WB
  }
  lastBucketTime = bucketTime;
}

// Main Setup /////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);
  espSerial.begin(9600);
  delay(2000);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  pinMode(rainPin, INPUT);
  digitalWrite(rainPin, LOW);  
  attachInterrupt(digitalPinToInterrupt(rainPin), countPulse, RISING); //Rain Gauge Interrupt

  water = 0;
  attachInterrupt(digitalPinToInterrupt(waterPin), waterPulse, RISING); //Water Flow Sensor Interrupt
}

// Main Loop /////////////////////////////////////////////////////////////////////////
void loop(){
  //Check for manual watering button press and activate relay
  if (buttonReleased) {
    buttonReleased = false;
    Serial.println("Manual Watering Activated!");
    digitalWrite(relayPin, HIGH);
    waterState = true;
    water = 0;
  }

  //Take Sensor Readings
  int chk;
  chk = DHT.read(DHT11_PIN);
  float h = DHT.humidity; //Read humidity
  float temperature = DHT.temperature; //Read temperature
  soilMoisture = analogRead(A11); //Read Soil moisture sensor measurement
  float soilMoisturePercent = -0.3571 * soilMoisture + 192.14; //Convert SM measurement to percentage

  //Print Readings to Serial
  Serial.print("H: ");
  Serial.print(h);
  Serial.print("% ");
  Serial.print(" T: ");
  Serial.print(temperature);
  Serial.print("C");
  Serial.print(" SM: ");
  Serial.print(soilMoisturePercent, 2);
  Serial.print(" WB: ");
  Serial.println(WB);


  //Maximum and Minimum Temperature Calculation
  if (temperature > maxTemp) {
    maxTemp = temperature;
  }
  if (temperature < minTemp) {
    minTemp = temperature;
  }

  //Maximum and Minimum Humidity Calculation
  if (h > maxHumidity) {
    maxHumidity = h;
  }
  if (h < minHumidity) {
    minHumidity = h;
  }

  // ET Calculation
    // if 24 hours have passed
  unsigned long dayCounter = millis();
  if (dayCounter - previousDayCounter >= dayInterval) {
    Serial.println("24 Hours have passed... Running Watering Checks:");
    //Check if Critical Soil Moisture
    if (soilMoisturePercent < 15) {
      Serial.println("Soil Moisture % is Critical, watering plant!");
      digitalWrite(relayPin, HIGH);
      waterState = true;
      water = 0;
    }
    else {
      Serial.println("Soil Moisture % is not critical, calculating ET:");
      float avgT = (maxTemp + minTemp)/2;
      float avgH = (maxHumidity + minHumidity)/2;
      float ETloss = 0.0118*pow((1-(avgH/100)),0.2)*pow((maxTemp-minTemp),0.3)*(8.744*sqrt(avgT+10)-40)+0.1*(avgT+20)*(1-(avgH/100));
      WB = WB - ETloss;
      Serial.print("WB: ");
      Serial.println(WB);
      if (WB < 0) {
        Serial.println("WB value is below Critical, watering plant!");
        digitalWrite(relayPin, HIGH);
        waterState = true;
        water = 0;
      }
      else {
        Serial.println("WB value is not critical, recalculating in 24 hours");
      }
    }
    previousDayCounter = dayCounter;
    maxTemp = 0;
    minTemp = 100;
    maxHumidity = 0;
    minHumidity = 100;
  }

  //Check if watering is complete
  if (soilMoisturePercent >= 60) {
    digitalWrite(relayPin, LOW);
    const float areaConstant = 59.17;
    WB = WB + (water * areaConstant);
    waterState = false;
    water = 0;
  }

  // Convert readings to a string
  String Str = String(temperature)+String(" ")+String(h)+String(" ")+String(soilMoisturePercent)+String(" ")+String(WB)+String(" ")+String(dayCounter/60000);

  // Send readings to ESP-01S as a string
  espSerial.println(Str);

  // Check for incoming data from ESP-01S
  if (espSerial.available() > 0) {
    String data = espSerial.readString();
    Serial.print("Received data: ");
    Serial.println(data);
    if (data == "MANUAL WATERING") {
      buttonReleasedInterrupt();
      data = "DONE";      
    }
  }

  delay(1000);
}

//Function to measure amount of water from flow sensor
void waterPulse() {
  if (waterState == true && (-0.3571 * analogRead(A11) + 192.14)) {
    water += 1.0/5880.0;
    Serial.println(water);
  }
}
