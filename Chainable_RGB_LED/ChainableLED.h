/*
Copyright (C) 2013 Seeed Technology Inc.
Copyright (C) 2012 Paulo Marques (pjp.marques@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
 * Library for controlling a chain of RGB LEDs based on the P9813 protocol.
 * E.g., supports the Grove Chainable RGB LED product.
 *
 * Information about the P9813 protocol obtained from:
 * http://www.seeedstudio.com/wiki/index.php?title=Twig_-_Chainable_RGB_LED
 */



#ifndef CHAINABLELED_H
#define CHAINABLELED_H

#include "mbed.h"

class ChainableLED
{
public:
    ChainableLED(PinName clk_pin, PinName data_pin, uint32_t number_of_leds);
    ~ChainableLED();

    void setColorRGB(uint32_t led, uint8_t red, uint8_t green, uint8_t blue);
    void setColorHSB(uint32_t led, float hue, float saturation, float brightness);
    void ledsOff(void);

private:
    DigitalOut _clk_pin;
    DigitalOut _data_pin;
    uint32_t   _num_leds;
    
    typedef union {
        uint8_t rgb[3];
        struct {
            uint8_t r, g, b;
        };
    } led_val_t;
    led_val_t *_leds;

    void sendByte(uint8_t b);
    void sendColor(uint8_t red, uint8_t green, uint8_t blue);
};

#endif
