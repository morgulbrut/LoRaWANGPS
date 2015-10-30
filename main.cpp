/*

Copyright (c) 2015, Tillo Bosshart
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of [project] nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


----- Lora Wan stack stuff
(C)2015 Semtech

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#include "TinyGPS.h"
#include "mbed.h"
#include "board.h"
#include "LoRaMac.h"
#include "utilities.h"
#include "ChainableLED.h"

#include <algorithm>

/*!
 * Mote device IEEE EUI
 */
static uint8_t DevEui[] = {
    0xf0, 0x3d, 0x29, 0x10, 0x00, 0x00, 0x16, 0x08
};


/*!
 * AES encryption/decryption cipher network session key
 */
static uint8_t NwkSKey[] = {
    0x94, 0x49, 0x1b, 0x94, 0x3a, 0x95, 0xe9, 0xe8,
    0xd5, 0xde, 0x91, 0xc2, 0xca, 0xbd, 0xdd, 0xc3

};

/*!
 * AES encryption/decryption cipher application session key
 */
static uint8_t AppSKey[] = {
    0x97, 0x74, 0x11, 0x8b, 0xa8, 0xd8, 0x22, 0x1c,
    0x65, 0x8f, 0x70, 0xb8, 0x0e, 0x55, 0xbd, 0x1c
};

/*!
 * Device address
 */
static uint32_t DevAddr = 0x80001608;

/*!
 * Indicates if the MAC layer has already joined a network.
 */
static bool IsNetworkJoined = false;

/*!
 * Defines the application data transmission duty cycle
 */
#define APP_TX_DUTYCYCLE                             5000000  // 6 [s] value in us
#define APP_TX_DUTYCYCLE_RND                         2000000  // 2 [s] value in us

/*!
 * User application data buffer size
 */
#define APP_DATA_SIZE                               30

/*!
 * User application data
 */
static uint8_t AppData[APP_DATA_SIZE];

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

Ticker TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
static bool TxNextPacket = true;
static bool TxDone = false;

static uint8_t AppPort = 3;
static uint8_t AppDataSize = APP_DATA_SIZE;

static LoRaMacEvent_t LoRaMacEvents;


bool trySendingFrameAgain;
/*!
* GPS related stuff
*/
TinyGPS gpsr;

Serial serial_gps(PB_10, PB_11);        // tx, rx
double lat, lon;
unsigned long age;
int year;
byte month, day, hour, minute, second;


#define NUM_LED 1

ChainableLED color_led(D7, D8, NUM_LED);
DigitalOut buzzer(A2);


/*!
 *  Payload...
 */
static void PrepareTxFrame( uint8_t port )
{


    char lat_ar[10];
    char lon_ar[10];
    char time_ar[10];
    int time = 1234567890;

    memset(lat_ar, 0x00, sizeof(lat_ar));
    sprintf(lat_ar, "%f", lat);
    memset(lon_ar, 0x00, sizeof(lon_ar));
    sprintf(lon_ar, "%f", lon);
    memset(time_ar, 0x00, sizeof(time_ar));
    sprintf(time_ar, "%d", time);

    std::copy(lat_ar, lat_ar + 10, AppData);
    std::copy(lon_ar, lon_ar + 10, AppData + 10);
    std::copy(time_ar, time_ar + 10, AppData + 20);



    //debug( "[Tx] LightValue=%d 0x%x\n\r" , tempValue, tempValue);

}

/*
* Process RX.
*
*/

static void ProcessRxFrame( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    debug( "[Rx] Port=%d\n\r" , info->RxPort);
    switch( info->RxPort ) { // Check Rx port number
        case 10:
            break;

        case 20:
            break;

        case 30:
            break;

        default:
            break;
    }
}


/*
* Only use
*
*/

static bool SendFrame( void )
{
    uint8_t sendFrameStatus = 0;

    sendFrameStatus = LoRaMacSendFrame( AppPort, AppData, AppDataSize );
//   sendFrameStatus = LoRaMacSendConfirmedFrame( AppPort, AppData, AppDataSize, 8 );
    switch( sendFrameStatus ) {
        case 5: // NO_FREE_CHANNEL
            // Try again later
            return true;
        default:
            return false;
    }
}


/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    TxNextPacket = true;
    TxNextPacketTimer.detach( );
}


/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    if( flags->Bits.JoinAccept == 1 ) {
        IsNetworkJoined = true;
    }

    if( flags->Bits.Tx == 1 ) {
    }

    if( flags->Bits.Rx == 1 ) {
        if( flags->Bits.RxData == true ) {
            ProcessRxFrame( flags, info );
        }
    }

    // Schedule a new transmission
    TxDone = true;
}

void gps_read()
{
    if (serial_gps.readable()) {
        while(serial_gps.readable()) {
            // debug("serial GPS readable\n");
            char c = serial_gps.getc();
            bool gps_available = gpsr.encode(c);
            if(gps_available) {
                color_led.setColorRGB( 0, 0,120,0);


                (void)gpsr.f_get_position(&lat, &lon, &age);
                (void)gpsr.crack_datetime(&year, &month, &day, &hour, &minute, &second);

                debug("%d-%02d-%02d %02d:%02d:%02d, ", year, month, day, hour, minute, second);
                debug("%f, %f\n", lat, lon);
            }
        }
    } else {
        color_led.setColorRGB( 0, 120,0,0);
    }
}

/**
*   Arduino style
*/

void setup()
{

    trySendingFrameAgain= false;

    buzzer = 0;         // 0: OFF,      1: ON

    debug( "\n\n\r    LoRaWAN Class A Demo code  \n\n\r" );

    BoardInitMcu( );
    BoardInitPeriph( );

    // Initialize LoRaMac device unique ID
    // BoardGetUniqueId( DevEui );

    LoRaMacEvents.MacEvent = OnMacEvent;
    LoRaMacInit( &LoRaMacEvents );

    IsNetworkJoined = false;

    TxNextPacket = true;

    LoRaMacSetAdrOn( true );

    LoRaMacSetDutyCycleOn( true );

}

void loop()
{
    while( IsNetworkJoined == false ) {
    }

    if( TxDone == true ) {

        TxDone = false;

        debug( "TxDone \n\n\r" );
        // Schedule next packet transmission
        TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        TxNextPacketTimer.attach_us( &OnTxNextPacketTimerEvent, TxDutyCycleTime );
    }

    if( trySendingFrameAgain == true ) {
        trySendingFrameAgain = SendFrame( );
    }

    if( TxNextPacket == true ) {
        TxNextPacketTimer.detach( );

        TxNextPacket = false;

        PrepareTxFrame( AppPort );

        trySendingFrameAgain = SendFrame( );
    }

    gps_read();

}



/**
 * Main application entry point.
 */
int main( void )
{
    setup();

    while( 1 ) {
        loop();
    }
}

