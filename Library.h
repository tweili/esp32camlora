#include <Arduino.h>
#include "FS.h"                 //SD Card ESP32
#include "soc/soc.h"            //disable brownout problems
#include "soc/rtc_cntl_reg.h"   //disable brownout problems
#include "driver/rtc_io.h"
#include <esp_camera.h>

#include <SPI.h>
#include <SX127XLT.h>           //get library here > https://github.com/StuartsProjects/SX12XX-LoRa
#include <ProgramLT_Definitions.h>

SX127XLT LoRa;          //create a library class instance called LoRa, needed for ARtransferIRQ.h

//#include "SDlibrary.h"
#include "Settings.h"   
#include <ARtransferIRQ.h>   //library of transfer functions
#include "DTSDlibrary.h"

#include "esp_camera.h"
char DTfilenamebuff[] = "/$50SATS.JPG"; //file length 6880 bytes, file CRC 0x0281

uint8_t *ptrDTsendarray;                //create a global pointer to the array to send, so all functions have access

void redFlash(uint16_t flashes, uint16_t ondelaymS, uint16_t offdelaymS);
void boot();
bool setupLoRaDevice();
bool configInitCamera();
bool WaitForArray();
bool sendArray(uint8_t *array, uint32_t arrayL);
bool takePhotoSend();

void redFlash(uint16_t flashes, uint16_t ondelaymS, uint16_t offdelaymS)
{
  uint16_t index;

  pinMode(REDLED, OUTPUT);                    //setup pin as output

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(REDLED, LOW);
    delay(ondelaymS);
    digitalWrite(REDLED, HIGH);
    delay(offdelaymS);
  }
  pinMode(REDLED, INPUT);                     //setup pin as input
}

void boot()
{
  redFlash(4, 125, 125);

  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector
  rtc_gpio_hold_dis(GPIO_NUM_4);
  rtc_gpio_hold_dis(GPIO_NUM_12);             //LoRa NSS back to normal control after sleep

  pinMode(2, INPUT_PULLUP);
  digitalWrite(NSS, HIGH);
  pinMode(NSS, OUTPUT);

  Monitorport.begin(115200);
  Monitorport.println();

  if (bootCount == 0)                         //run this only the first time after programming or power up
  {
    bootCount = bootCount + 1;
  }

  Monitorport.println(F("Awake !"));
  Monitorport.print(F("Bootcount "));
  Monitorport.println(bootCount);
  Monitorport.print(F("Sleepcount "));
  Monitorport.println(sleepcount);

#ifdef DISABLEPAYLOADCRC
  LoRa.setReliableConfig(NoReliableCRC);
#endif

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    Monitorport.println(F("Payload CRC disabled"));
  }
  else
  {
    Monitorport.println(F("Payload CRC enabled"));
  }

  if (psramInit())
  {
    Monitorport.println("PSRAM is initialised");
    available_PSRAM = ESP.getFreePsram();
    Monitorport.print("PSRAM available: ");
    Monitorport.print(available_PSRAM);                                                    
    Monitorport.println(" bytes");
  }
  else
  {
    Monitorport.println("PSRAM not available");
    Monitorport.println("Program halted");
    while (1);
  }

  allocated_PSRAM = available_PSRAM / 2;                 //dont use all PSRAM for array

  Monitorport.print("Allocate ");
  Monitorport.print(allocated_PSRAM);
  Monitorport.println(" bytes of PSRAM as receive array");

  uint8_t *byte_array = (uint8_t *) ps_malloc((allocated_PSRAM) * sizeof(uint8_t));
  PSRAMptr = byte_array;                                 //save the pointer to byte_array as global pointer

  if (!configInitCamera())
  {
    Monitorport.println(F("Camera config failed"));
    while (1) redFlash(100, 25, 25);
  }
}

bool setupLoRaDevice()
{
  SPI.begin(SCK, MISO, MOSI, NSS);

  if (LoRa.begin(NSS, NRESET, LORA_DEVICE))
  //if (LoRa.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    Monitorport.println(F("LoRa device found"));
  }
  else
  {
    Monitorport.println(F("LoRa Device error"));
    return false;
  }                                                                                        
  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);   
#ifdef DISABLEPAYLOADCRC
  LoRa.setReliableConfig(NoReliableCRC);
#endif

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    Monitorport.println(F("Payload CRC disabled"));
  }
  else
  {
    Monitorport.println(F("Payload CRC enabled"));
  }
  return true;
}

bool configInitCamera()
{
  Serial.println(F("Initialising the camera module "));

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;      //YUV422,GRAYSCALE,RGB565,JPEG

  //Select lower framesize if the camera doesn't support PSRAM
  if (psramFound())
  {
    Serial.println(F("PSRAM found"));
    config.frame_size = FRAMESIZE_UXGA;      //FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 2;                 //0-63 lower number means higher quality
    config.fb_count = 2;

    // approximate file sizes
    // @ jpeg_quality = 10, SVGA image size = 25K+, UXGA image size = 55K+,
    // @ jpeg_quality = 2, SVGA image size = 60K+, UXGA image size = 160K+,                   // image sizes of circa 200K+ can cause ESP32CAM camera code to crash
  }
  else
  {
    Serial.println(F("No PSRAM"));
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);   //Initialize the Camera
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 1);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 450);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 0);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  return true;
}

bool WaitForArray()
{
  setupLoRaDevice();
  Monitorport.println(F("LoRa file transfer receiver ready"));

  ARreceiveArrayL = ARreceiveArrayNoACK(PSRAMptr, allocated_PSRAM, ReceiveTimeoutmS);

  SPI.end();
  digitalWrite(NSS, HIGH);
  digitalWrite(NRESET, HIGH);
  delay(500);

  if (ARreceiveArrayL)
  {
    Monitorport.print(F("Received array length "));
    Monitorport.println(ARreceiveArrayL);
    return true;
  }
  else
  {
    Monitorport.println(F("Error receiving array"));
    if (ARDTArrayTimeout)
    {
      Monitorport.println(F("Timeout receiving array"));
    }
    return false;
  }
}

bool sendArray(uint8_t *array, uint32_t arrayL)
{
  char command[] = "CMD";
  bool sentOK = false;

  if (setupLoRaDevice())
  {
    Monitorport.print(F("Send with LoRa "));
    Monitorport.println(command);
  } 
  else
  {
    Monitorport.println(F("LoRa device not available"));
    return sentOK;
  }

  pinMode(REDLED, OUTPUT);
  digitalWrite(REDLED,LOW);
  sentOK = ARsendArrayNoACK(array, arrayL, command, strlen(command));
  digitalWrite(REDLED,HIGH);
  pinMode(REDLED, INPUT);  
  SPI.end();
  digitalWrite(NSS, HIGH);
  digitalWrite(NRESET, HIGH);

  if (sentOK)
  {
    Monitorport.print(command);
    Monitorport.println(F(" Sent array OK"));
  }
  else
  {
    Monitorport.print(command);
    Monitorport.println(F(" Send array failed"));
  }
  return sentOK;
}

uint32_t moveFileArray(char *filenamebuff, uint8_t *buff, uint32_t buffsize)
{
  uint32_t index;

  ptrDTsendarray = buff;                                         //assign passed array ptr to global ptr

  if (DTSD_initSD())
  {
#ifdef ENABLEMONITOR
    Monitorport.println(F("SD Card initialized."));
#endif
  }
  else
  {
    Monitorport.println(F("SD Card failed, or not present."));
    while (1) redFlash(100, 25, 25);
  }

  DTLocalFileLength = DTSD_getFileSize(filenamebuff);            //get the file length

  if (DTLocalFileLength == 0)
  {
    Monitorport.print(F("Error - opening local file "));
    Monitorport.println(filenamebuff);
    return 0;
  }

  if (DTLocalFileLength > buffsize)
  {
    Monitorport.println(filenamebuff);
    Monitorport.print(F("Error - file length of "));
    Monitorport.print(DTLocalFileLength);
    Monitorport.print(F(" bytes exceeds array length of "));
    Monitorport.print(buffsize);
    Monitorport.println(F(" bytes"));
    return 0;
  }

  DTSD_openFileRead(filenamebuff);
#ifdef ENABLEMONITOR
  Monitorport.print(F("Opened local file "));
  Monitorport.print(filenamebuff);
  Monitorport.print(F(" "));
  Monitorport.print(DTLocalFileLength);
  Monitorport.println(F(" bytes"));
#endif
  DTLocalFileCRC = DTSD_fileCRCCCITT(DTLocalFileLength);                   //get file CRC from position 0 to end
#ifdef ENABLEMONITOR
  Monitorport.print(F("DTLocalFileCRC 0x"));
  Monitorport.println(DTLocalFileCRC, HEX);
#endif

  //now tranfer SD file to global array
  dataFile.seek(0);                                                        //ensure at first position in file
  for (index = 0; index < DTLocalFileLength; index++)
  {
    buff[index] = dataFile.read();
  }

  DTSD_closeFile();

#ifdef ENABLEMONITOR
  Monitorport.println(F("DTsendarray loaded from SD"));
  Monitorport.print(F("Last written location "));
  Monitorport.println(index);
  Monitorport.print(F("First 16 bytes of array to send "));
  LoRa.printHEXPacket(buff, 16);
  Monitorport.println();
#endif
  DTarraylocation = 0;
  return DTLocalFileLength;
}

bool takePhotoSend()
{
  bool sentOK = false;

  camera_fb_t  * fb = esp_camera_fb_get();

  if (!fb)
  {
    Monitorport.println(F("*****************************"));
    Monitorport.println(F("ERROR - Camera capture failed"));
    Monitorport.println(F("*****************************"));
  } else {
    Monitorport.println(F("Camera capture success"));
  }


  if (setupLoRaDevice())
  {
    Monitorport.print(F("Send Photo with LoRa"));
  }
  else
  {
    Monitorport.println(F("LoRa device not available"));
    return sentOK;
  }

  pinMode(REDLED, OUTPUT);                    //setup pin as output 1
  digitalWrite(REDLED, LOW);
  sentOK = ARsendArray(fb->buf, fb->len, DTfilenamebuff, strlen(DTfilenamebuff)); //pass array pointer and length across to LoRa send function
  digitalWrite(REDLED, HIGH);
  pinMode(REDLED, INPUT);                    //setup pin as output 1

  esp_camera_fb_return(fb);                                    //return the frame buffer back to the driver for reuse

  SPI.end();
  digitalWrite(NSS, HIGH);
  digitalWrite(NRESET, HIGH);

  if (sentOK)
  {
    Monitorport.print(DTfilenamebuff);
    Monitorport.println(F(" Sent OK"));
  }
  else
  {
    Monitorport.print(DTfilenamebuff);
    Monitorport.println(F(" Send picture failed"));
  }
  return sentOK;
}

