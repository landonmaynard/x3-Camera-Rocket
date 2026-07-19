#include <Arduino.h>
#include <SparkFunBME280.h>
#include <esp_camera.h>
#include <Wire.h>
#include <FS.h>
#include <SD_MMC.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

BME280 bmp;

TaskHandle_t bmpTask;

//STATES
volatile bool beforeFlight = true;
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
int landingDetectionDelay = 1000 / landingDetectionHZ; //how much to wait in between readings for landing detection
unsigned long landingPrevMillis;

//saving altitudes to ram
struct bmpData{
  float altF;
  float tempF;
  float presP;
  unsigned long timeM;
};
bmpData* bmpDataArray = NULL;
const int bmpDataArraySize = 80000;
int currentIndex = 0;
int bmpDataHZ = 60; //how many times per second to save altitude data to ram
int bmpDataDelay = 1000/bmpDataHZ; //how many milliseconds to wait between saving data to ram
int bmpPreviousMillis = 0;

//video vars
File videoFile;
bool recording = false;
bool recordingDescent = false;
unsigned long lastFrameMillis = 0;
int videoFPS = 25;
unsigned long videoFrameDelay = 1000/videoFPS; //how many milliseconds to wait between frames

// put function declarations here:
bool detectLaunch();
bool detectApogee();
bool detectLanding();

void bmpTaskCode(void *parameter);


void setup() {
  // put your setup code here, to run once:

  Wire.begin(1, 3);
  bmp.beginI2C();

WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  psramInit();

camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 10;
  config.fb_count = 4;
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);

  if (err == ESP_OK) {
  sensor_t * s = esp_camera_sensor_get();
  // Drop down to VGA for the high-FPS ascent video recording
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_brightness(s, -1);     // -2 to 2 **************************
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable 
  }

  SD_MMC.begin("/sdcard", false);
  uint8_t cardType = SD_MMC.cardType();

  camera_fb_t * fb = NULL;

  bmpDataArray = (bmpData*)ps_malloc(sizeof(bmpData) * bmpDataArraySize);
  xTaskCreatePinnedToCore(bmpTaskCode, "bmpTask", 10000, NULL, 0, &bmpTask, 0);
  pinMode(2, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(beforeFlight && !recording && !apogeeDetected){
    videoFile = SD_MMC.open("/ascent.mjpeg", FILE_WRITE);
    if(videoFile){
      recording = true;
      lastFrameMillis = millis();
    }
  }
  if(recording && !apogeeDetected){
    if(millis() - lastFrameMillis >= videoFrameDelay){
      lastFrameMillis = millis();
      camera_fb_t * fb = esp_camera_fb_get();
      if(fb){
        videoFile.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);
      }
    }
  }
  if(apogeeDetected && recording){
    videoFile.close();
    recording = false;

    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_UXGA); //return to UXGA for the high-res descent video recording
    videoFPS=5;
    videoFrameDelay = 1000/videoFPS; //how many milliseconds to wait between frames
    delay(250); //wait for camera to adjust to new resolution
  }
  if(apogeeDetected && !recordingDescent && !landingDetected){
    videoFile = SD_MMC.open("/descent.mjpeg", FILE_WRITE);
    if(videoFile){
      recordingDescent = true;
      lastFrameMillis = millis();
    }
  }
  if(recordingDescent && !landingDetected){
    if(millis() - lastFrameMillis >= videoFrameDelay){
      lastFrameMillis = millis();
      camera_fb_t * fb = esp_camera_fb_get();
      if(fb){
        videoFile.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);
      }
    }
  }
  if(landingDetected && recordingDescent){
    videoFile.close();
    recordingDescent = false;
    SD_MMC.end();
  }
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
  if(previousAlt - currentAlt > apogeeDetectionThreshold){
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
      currentAlt = 0;
      for(int i=0; i<altArraySize; i++){
        currentAlt += altArray[i];
      }
      currentAlt /= altArraySize;
      previousMillis = millis();
    }

    //Pass flight stages to main loop

    if(beforeFlight){
      if(detectLaunch()){
        beforeFlight = false;
      }
    }
    if(!beforeFlight && !apogeeDetected){
      if(detectApogee()){
        apogeeDetected = true;
      }
    }
    if(!beforeFlight && apogeeDetected && !landingDetected){
      if(detectLanding()){
        landingDetected = true;
      }
    }

    //Writing alt to ram
    if(!landingDetected){
      if(millis() - bmpPreviousMillis >= bmpDataDelay){
        bmpDataArray[currentIndex].altF = bmp.readFloatAltitudeFeet();;
        bmpDataArray[currentIndex].tempF = bmp.readTempF();
        bmpDataArray[currentIndex].presP = bmp.readFloatPressure();
        bmpDataArray[currentIndex].timeM = millis();
        currentIndex++;
        if(currentIndex >= bmpDataArraySize){
          currentIndex = 0; //wrap around to the beginning of the array
        }
        bmpPreviousMillis = millis();
      }
    }
    if(landingDetected){
      vTaskDelay(1000 / portTICK_PERIOD_MS); //wait a second to make sure the last reading is saved to ram
      File bmpDataFile = SD_MMC.open("/bmpData.csv", FILE_WRITE);
      if(bmpDataFile){
        bmpDataFile.println("Altitude (ft),Temperature (F),Pressure (Pa),Time (ms)");
        for(int i=0; i<currentIndex; i++){
          bmpDataFile.print(bmpDataArray[i].altF);
          bmpDataFile.print(",");
          bmpDataFile.print(bmpDataArray[i].tempF);
          bmpDataFile.print(",");
          bmpDataFile.print(bmpDataArray[i].presP);
          bmpDataFile.print(",");
          bmpDataFile.println(bmpDataArray[i].timeM);
        }
        bmpDataFile.close();
      }
      vTaskDelete(NULL); //delete this task since it is no longer needed
    } 
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
