/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"

Timer TimeCounter;
Ticker LoadTimeCounter;


volatile uint32_t currentTime = 0;

void ResetTimecounter( void )
{
    currentTime = currentTime + TimeCounter.read_us();
    TimeCounter.reset();
    TimeCounter.start( );
}

void TimerHwInit( void )
{
    TimeCounter.start( );
    LoadTimeCounter.attach( &ResetTimecounter, 10 );
}

TimerTime_t TimerGetCurrentTime( void )
{
    currentTime += TimeCounter.read_us();
    TimeCounter.reset();
    TimeCounter.start( );
    return ( ( TimerTime_t )currentTime );
}

