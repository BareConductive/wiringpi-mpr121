[![Bare Conductive](http://bareconductive.com/assets/images/LOGO_256x106.png)](http://www.bareconductive.com/)

# Bare Conductive MPR121 Wiring Pi Library

A fairly feature complete library for the Resurgent Semiconductor [MPR121QR2](http://www.nxp.com/files/sensors/doc/data_sheet/MPR121.pdf). This is a popular capacitive touch sense IC featuring:

* 12 electrodes/capacitance sensing inputs in which 8 are 
multifunctional for LED driving and GPIO
* Integrated independent autocalibration for each electrode input
* Autoconfiguration of charge current and charge time for each 
electrode input
* Separate touch and release trip thresholds for each electrode, 
providing hysteresis and electrode independence 
* I2C interface, with IRQ Interrupt output to advise electrode status 
changes

This library was originally developed to support the [Bare Conductive Pi Cap](http://www.bareconductive.com/shop/pi-cap/). However, it should work fine (with adaptations) for any Raspberry Pi connected to an MPR121.

Library includes support for:

* capacitive touch sensing
* proximity mode for virtual 13th electrode
* readback of filtered capacitance measurements
* full GPIO including PWM write

Still to be implemented:

* full support for autocalibration / autoconfig
* callback attachment for interrupt pin

## Requirements

* Requires [WiringPi](http://wiringpi.com/) (`apt-get install wiringpi`)
* before you use any of the MPR121 methods, you need to call its MPR121.begin() method, normally in setup()
* the default MPR121 I2C address for this library is 0x5C, which is used if you call MPR121.begin()
* if you want to specify a different address you can, with MPR121.begin(yourAddress)



## Install

* You should install this code as part of the Pi Cap Raspbian package: `sudo apt-get install picap`    
* However, if you are doing this yourself, clone the repository, enter it and run `make clean && make && make install`
* Note that you must run `make install` as root, so you might need to prepend the command with sudo


## Usage

```
// 0x5A for a lot of boards out there
// 0x5C for the Pi Cap

#define MPR121addr 0x5A 

#include <MPR121.h>

void setup(){
	MPR121.begin(MPR121addr);
}

void loop(){
	// your code goes here
}

int main(void) {
	setup();
	while(1){
		loop;
	}
	return(0);
}
```
