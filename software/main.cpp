#include <Arduino.h>
#include <SparkFunBME280.h>
#include <esp_camera.h>
#include <Wire.h>

BME280 bmp;

TaskHandle_t bmpTask;

//STATES
volatile bool onGround=true;
volatile bool apogeeDetected;
volatile bool landingDetected;

//BMP VARS
const int altArraySize = 50;
float altArray[altArraySize];
int altIndex = 0;
int previousIndex = 0;
float baselineAlt;
float currentAlt; //averaged
float previousAlt; //averaged
int bmpHZ = 120;
int detectionTime=3; //seconds
//in feet
int launchDetectionThreshold = 10; //if it rises this many feet in the detection time, it is considered launch
int apogeeDetectionThreshold = 10; //if it drops this many feet in the detection time, it is considered apogee
int landingDetectionWindow = 10; //stays within this many feet for the landing detection time to be considered landed
int landingDetectionTime = 5; //seconds
int landingDetectionHZ = 5; //checks x times per second in the span of landingDetectionTime
int landingDetectionIndex = 0; //index for landing detection counter
int landingDetectionAmount = landingDetectionTime * landingDetectionHZ; //convert seconds to number of readings
int landingDetectionDelay = landingDetectionHZ/1000; //how much to wait in between readings for landing detection
unsigned long landingPrevMillis;

//saving altitudes to ram

struct bmpData{
  float altF;
  float tempF;
  float presP;
  unsigned long timeM;
};
bmpData* bmpDataArray = NULL;
const int bmpDataArraySize = 40000;
int currentIndex = 0;
int bmpDataHZ = 60; //how many times per second to save altitude data to ram
int bmpDataDelay = 1000/bmpDataHZ; //how many milliseconds to wait between saving data to ram

// put function declarations here:
bool detectLaunch();
bool detectApogee();
bool detectLanding();


void setup() {
  // put your setup code here, to run once:
  psramInit();
  bmpDataArray = (bmpData*)ps_malloc(sizeof(bmpData) * bmpDataArraySize);
  void bmpTaskCode(void *parameter);
  xTaskCreatePinnedToCore(bmpTaskCode, "bmpTask", 10000, NULL, 0, &bmpTask, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Camera task
}

// put function definitions here:

bool detectLaunch(){
  if(currentAlt - baselineAlt > launchDetectionThreshold){
    return true;
  }else{
    return false;
  }
}

bool detectApogee(){
  if(previousAlt - currentAlt < apogeeDetectionThreshold){
    return true;
  }else{
    return false;
  }
}

bool detectLanding(){
  if(millis() - landingPrevMillis >= landingDetectionDelay){
    landingPrevMillis = millis();
    if(abs(previousAlt - currentAlt) < landingDetectionWindow){
      landingDetectionIndex++;
      if(landingDetectionIndex >= landingDetectionAmount){
        landingDetectionIndex = 0;
        return true;
      }
    }else{
      landingDetectionIndex = 0; //if it goes outside the window, reset the counter
    }
  }
  return false;
}

void bmpTaskCode(void *parameter){
// Establishes baseline altitude by averaging the first 50 readings, sets up array to prevent misreads
  for(int i=0; i<altArraySize; i++){
    altArray[i] = bmp.readFloatAltitudeFeet();
  }
  for(int i=0; i<altArraySize; i++){
    baselineAlt += altArray[i];
  }
  baselineAlt /= altArraySize;
 
  //Math that only needs to be done once
  unsigned long previousMillis = millis();
  landingPrevMillis = previousMillis;
  int previousAltDelay = detectionTime * (bmpHZ/60); //convert seconds to number of readings
  
  while(true){

    //get altitude for state machine

    if(millis() - previousMillis >= 1000/bmpHZ){
      altArray[altIndex] = bmp.readFloatAltitudeFeet();
      altIndex++;
      if(altIndex >= altArraySize){
        //half a second has passed
        altIndex = 0;
        previousIndex++;
        if(previousIndex >= previousAltDelay){
          previousIndex = 0;
          previousAlt = currentAlt; //creates a snapshot every detectionTime seconds to compare to current altitude
        }
      }
      for(int i=0; i<altArraySize; i++){
        currentAlt += altArray[i];
      }
      currentAlt /= altArraySize;
      previousMillis = millis();
    }

    //Pass flight stages to main loop

    if(onGround){
      if(detectLaunch()){
        onGround = false;
      }
    }
    if(!onGround){
      if(detectApogee()){
        apogeeDetected = true;
      }
    }
    if(!onGround && apogeeDetected){
      if(detectLanding()){
        landingDetected = true;
      }
    }

    //Writing alt to ram
    if(millis() - previousMillis >= bmpDataDelay){
      bmpDataArray[currentIndex].altF = bmp.readFloatAltitudeFeet();;
      bmpDataArray[currentIndex].tempF = bmp.readTempF();
      bmpDataArray[currentIndex].presP = bmp.readFloatPressure();
      bmpDataArray[currentIndex].timeM = millis();
      currentIndex++;
      if(currentIndex >= bmpDataArraySize){
        currentIndex = 0; //wrap around to the beginning of the array
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
