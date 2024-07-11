#include <Arduino.h>
#include "FS.h"                 //SD Card ESP32
#include "soc/soc.h"            //disable brownout problems
#include "soc/rtc_cntl_reg.h"   //disable brownout problems
#include "driver/rtc_io.h"

#include <SPI.h>
#include <SX127XLT.h>           //get library here > https://github.com/StuartsProjects/SX12XX-LoRa
#include <ProgramLT_Definitions.h>

SX127XLT LoRa;          //create a library class instance called LoRa, needed for ARtransferIRQ.h

#define SD SD_MMC
//#include "SDlibrary.h"

#define ENABLEMONITOR   //enable define to see progress messages in ARtransferIRQ.h
#define PRINTSEGMENTNUM //enable to print segment numbers as transfer progresses
#define ENABLEARRAYCRC  //enable this define to use and show CRCs
//#define DISABLEPAYLOADCRC //enable this define if you want to not use packet payload CRC checking                                                              
#define DEBUG           //enable this define to show data transfer debug info                                                                                  
RTC_DATA_ATTR int16_t bootCount = 0;             //variables to save in RTC ram
RTC_DATA_ATTR uint16_t sleepcount = 0;

#include <esp_camera.h>
camera_config_t config; //stores the camera configuration parameters
//#include "Transfer.h"   //library of transfer functions

uint8_t *PSRAMptr;      //create a global pointer to the array to send, so all functions have access
uint32_t available_PSRAM;
uint32_t allocated_PSRAM;

#define NSS 12         //select on LoRa device
#define NRESET 15      //reset pin on LoRa device
#define SCK 14         //SCK on SPI3
#define MISO 13        //MISO on SPI3
#define MOSI 2         //MOSI on SPI3
#define DIO0 4         //DIO0 on SPI3

#define REDLED 33      //pin number for ESP32CAM on board red LED, set logic level low for on
#define WHITELED 4     //pin number for ESP32CAM on board white LED

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using
#define Monitorport Serial                      //Serial port for prints

void boot();
bool setupLoRaDevice();
bool WaitForArray();
void redFlash(uint16_t flashes, uint16_t ondelaymS, uint16_t offdelaymS);

//*******  Setup LoRa modem parameters here ! ***************
const uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_500;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto                                                                            

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

bool WaitForArray()
{
  setupLoRaDevice();
  Monitorport.println(F("LoRa file transfer receiver ready"));

//  DTreceiveArrayL = ARreceiveArray(PSRAMptr, allocated_PSRAM, ReceiveTimeoutmS);

  SPI.end();
  digitalWrite(NSS, HIGH);
  digitalWrite(NRESET, HIGH);
  delay(500);

//  if (DTreceiveArrayL)
//  {
//    Monitorport.print(F("Received array length "));
//    Monitorport.println(DTreceiveArrayL);
//    return true;
//  }
//  else
//  {
//    Monitorport.println(F("Error receiving array"));
//    if (ARDTArrayTimeout)
//    {
//      Monitorport.println(F("Timeout receiving array"));
//    }
//    return false;
//  }
}


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


