# DK-SpectraBloom
_SpectraBloom_ is a real-time audio spectrum visualizer with music-reactive effects. The front spectrum display analyzes 16 bands of audio, from sub-bass to 20,000Hz. The twin LED modules on top display 16 randomized algorithms reacting to the audio spectrum. 

![Spectra Bloom Visualizer Front](/images/spectra_bloom_front.jpg)

## About
This is one of my _[Spectra](https://github.com/search?q=user%3Adkrue+spectra)_ series audio visualizer projects. This project is available prebuilt in my 
[Etsy Store: Circuits & Sawdust](https://www.etsy.com/listing/508716612/spectra-spectrum-analyzer-music-light).

## Goal
My main goal with this project was to find out if your typical Arduino 16MHz _ATmega 328P_ chip could analyze 16 bands of audio smoothly. All the examples I could find only analyzed 8 bands, plus I wanted to use the audio analysis results to power an LED-based light show. I happily met that goal with this project.

![Spectra Bloom Visualizer Side](/images/spectra_bloom_side.jpg)

## How it works
This is an [Adafruit Pro Trinket](https://www.adafruit.com/product/2000) (5V) based project. The Pro Trinket is great because it's cheap but still offers an analog reference pin to tie line-level audio into. It's 16MHz and fast enough to analyze 16 bands of FFT audio, but it has no serial to USB chip, so I recommend breadboarding this project out on an Arduino Uno. You can use the serial output for debugging, and then transfer everything to the Pro Trinket. Or you could use something like the [Adafruit Metro Mini](https://www.adafruit.com/product/2590) for a similar cost.

The original inspiration for this is the [Adafruit Piccolo](https://learn.adafruit.com/piccolo/overview) project. Rather than rely on a variable-level microphone input for the source audio, I addded a 3.5mm audio input jack with voltage divider circuit to analyze line-level audio. A second 3.5mm output jack is tied to the input jack to pass through audio to other modules.  I found this [spectrum analyzer Instructables](https://www.instructables.com/id/Arduino-Spectrum-Analyzer-on-a-10x10-RGB-LED-Matri/) to be helpful for information on how to wire in audio jacks.

## Ingredients
This project uses premium parts from Adafruit. Check out my other [Spectra projects](https://github.com/search?q=user%3Adkrue+spectra) for some low-cost alternatives using more common Chinese components.

- Two [bi-color square pixel LED matrix with I2C backpacks](https://www.adafruit.com/product/902)
- Two RGB 5050 LED [Neopixel Jewels](https://www.adafruit.com/product/2226)
- [Adafruit Pro Trinket](https://www.adafruit.com/product/2000)
- A shiny [upcycled project enclosure](https://www.ebay.com/itm/292067232173)

![Spectra Bloom Visualizer Dark](/images/spectra_bloom_dark.jpg)
