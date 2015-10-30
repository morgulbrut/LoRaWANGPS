/*
 * Copyright (C) 2013 Seeed Technology Inc.
 * Copyright (C) 2012 Paulo Marques (pjp.marques@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* Information about the P9813 protocol obtained from:
 * http://www.seeedstudio.com/wiki/index.php?title=Twig_-_Chainable_RGB_LED
 *
 * HSB to RGB routine adapted from:
 * http://mjijackson.com/2008/02/rgb-to-hsl-and-rgb-to-hsv-color-model-conversion-algorithms-in-javascript
 *
 * This library is ported from Arduino to mbed
 */

#include "ChainableLED.h"

float hue2rgb(float p, float q, float t)
{
    if (t < 0.0f) {
        t += 1.0f;
    }
    if(t > 1.0f) {
        t -= 1.0f;
    }
    if(t < 1.0f/6.0f) {
        return p + (q - p) * 6.0f * t;
    }
    if(t < 1.0f/2.0f) {
        return q;
    }
    if(t < 2.0f/3.0f) {
        return p + (q - p) * (2.0f/3.0f - t) * 6.0f;
    }

    return p;
}

// --------------------------------------------------------------------------------------

ChainableLED::ChainableLED(PinName clk_pin, PinName data_pin, uint32_t number_of_leds) :
    _clk_pin(clk_pin),
    _data_pin(data_pin),
    _num_leds(number_of_leds)
{
    _leds = new led_val_t [number_of_leds];
    _clk_pin = 0;
    _data_pin = 0;
    ledsOff();
}

ChainableLED::~ChainableLED()
{
    delete [] _leds;
}

void ChainableLED::ledsOff(void)
{
    for (uint8_t i=0; i<_num_leds; i++) {
        setColorRGB(i, 0, 0, 0);
    }
}

void ChainableLED::sendByte(uint8_t b)
{
    // Send one bit at a time, starting with the MSB
    for (uint8_t i=0; i<8; i++) {
        _data_pin = (b & 0x80) ? 1 : 0;
        _clk_pin = 1;
        _clk_pin = 0;
        b <<= 1;
    }
}

void ChainableLED::sendColor(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t r67 = ~(red & 0xC0) >> 6;;
    uint8_t g67 = ~(green & 0xC0) >> 4;
    uint8_t b67 = ~(blue & 0xC0) >> 2;
    uint8_t verify = 0xC0 | r67 | g67 | b67;
    
    sendByte(verify);
    sendByte(blue);
    sendByte(green);
    sendByte(red);
}

void ChainableLED::setColorRGB(uint32_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    // Send data frame prefix (32x "0")
    sendByte(0x00);
    sendByte(0x00);
    sendByte(0x00);
    sendByte(0x00);

    // Send color data for each one of the leds
    for (uint8_t i=0; i<_num_leds; i++) {
        if (i == led) {
            _leds->rgb[i*3 + 0] = red;
            _leds->rgb[i*3 + 1] = green;
            _leds->rgb[i*3 + 2] = blue;
        }
        sendColor(_leds->rgb[i*3 + 0], _leds->rgb[i*3 + 1], _leds->rgb[i*3 + 2]);
    }

    // Terminate data frame (32x "0")
    sendByte(0x00);
    sendByte(0x00);
    sendByte(0x00);
    sendByte(0x00);
}

template<typename T>
T min(T a, T b) { return (a < b) ? a : b; }

template<typename T>
static inline T max(T a, T b) { return (a > b) ? a : b; }

void ChainableLED::setColorHSB(uint32_t led, float hue, float saturation, float brightness)
{
    float r, g, b;
    
    hue = min<float>(hue, 1.0f);//constrain(hue, 0.0, 1.0);
    hue = max<float>(hue, 0.0f);
    saturation = min<float>(saturation, 1.0f);//constrain(saturation, 0.0, 1.0);
    saturation = max<float>(saturation, 0.0f);
    brightness = min<float>(brightness, 1.0f);//constrain(brightness, 0.0, 1.0);
    brightness = max<float>(brightness, 0.0f);

    if(saturation == 0.0f) {
        r = g = b = brightness;
    } else {
        float q = (brightness < 0.5f) ? (brightness * (1.0f + saturation)) : (brightness + saturation - brightness * saturation);
        float p = 2.0f * brightness - q;
        r = hue2rgb(p, q, hue + 1.0f/3.0f);
        g = hue2rgb(p, q, hue);
        b = hue2rgb(p, q, hue - 1.0f/3.0f);
    }

    setColorRGB(led, (uint8_t)(255.0f*r), (uint8_t)(255.0f*g), (uint8_t)(255.0f*b));
}
