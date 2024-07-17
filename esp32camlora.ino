#include "Library.h"


void setup()
{
    boot();
}

void loop()
{
// ***** Receiver ******
//    WaitForArray();

// ***** Transmitter ***

// *** 1. send array

//    uint8_t commandarray[] = {
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
//    0x02,0x01,0x02,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01
//    };
//    sendArray(commandarray, 260);

// *** 2. send array from SD

//    uint8_t DTsendArray[0x10000];   //create a global array to hold data to transfer
//    uint32_t DTsendArrayL;
//
//    DTsendArrayL = moveFileArray(DTfilenamebuff, DTsendArray, sizeof(DTsendArray)); //move the file to a global array to be sent
//
//    if (DTsendArrayL == 0)
//    {
//      Monitorport.println("ERROR moving local file to array - program halted");
//      while (1);
//    }
//    sendArray(DTsendArray, DTsendArrayL);

// *** 3. send photo

    takePhtoSend();
    delay(10000);
}
