/*******************************************************************************

  Bare Conductive MPR121 library
  ------------------------------

  MPR121.cpp - MPR121 class implementation file

  Based on code by Jim Lindblom and plenty of inspiration from the Freescale
  Semiconductor datasheets and application notes.

  Bare Conductive code written by Stefan Dzisiewski-Smith, Peter Krige and
  Szymon Kaliski.

  This work is licensed under a MIT license https://opensource.org/licenses/MIT
  
  Copyright (c) 2016, Bare Conductive
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

 *******************************************************************************/

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include <stdint.h>
  #include <stdio.h>
  #include <sys/stat.h>
}

#include "MPR121.h"

#define DEBUG 0
#define NOT_INITED_BIT 0
#define ADDRESS_UNKNOWN_BIT 1
#define READBACK_FAIL_BIT 2
#define OVERCURRENT_FLAG_BIT 3
#define OUT_OF_RANGE_BIT 4

MPR121_t::MPR121_t(){
  address = 0x5C;    // default address is 0x5C, for use with Bare Touch Board
  ECR_backup = 0x00;
  running = false;
  error = 1<<NOT_INITED_BIT; // initially, we're not initialised
  touchData = 0;
  lastTouchData = 0;
  autoTouchStatusFlag = false;
}

void MPR121_t::setRegister(uint8_t reg, uint8_t value){

  bool wasRunning = false;

  if(reg==MPR121_ECR){	// if we are modding MPR121_ECR, update our internal running status
    if(value&0x3F){
      running = true;
    } else {
      running = false;
    }
  } else if(reg<MPR121_CTL0){
    wasRunning = running;
    if(wasRunning) stop(); 	// we should ALWAYS be in stop mode for this
    // unless modding MPR121_ECR or GPIO / LED register
  }

  wiringPiI2CWriteReg8 (fd, reg, value);

  if(wasRunning) run();		// restore run mode if necessary
}

uint8_t MPR121_t::getRegister(uint8_t reg){
  uint8_t scratch;

  scratch = wiringPiI2CReadReg8(fd, reg);
  // auto update errors for registers with error data
  if(reg == MPR121_TS2 && ((scratch&0x80)!=0)){
    error |= 1<<OVERCURRENT_FLAG_BIT;
  } else {
    error &= ~(1<<OVERCURRENT_FLAG_BIT);
  }
  if((reg == MPR121_OORS1 || reg == MPR121_OORS2) && (scratch!=0)){
    error |= 1<<OUT_OF_RANGE_BIT;
  } else {
    error &= ~(1<<OUT_OF_RANGE_BIT);
  }
  return scratch;
}

bool MPR121_t::begin(){
  fd = wiringPiI2CSetup(this->address);

  if(!wiringPiInited){
    // default behaviour is stop-then-start - we want repeated starts
    // we also only need to do this once
    enableRepeatedStarts();

    if(wiringPiSetup() != -1){ // weird, but apparently we look for "not -1" (as opposed to 0) for success
      wiringPiInited = true;
    }
  }
  error &= ~(1<<NOT_INITED_BIT); // clear NOT_INITED error flag

  if(reset()){
    // default values...
    applySettings(&defaultSettings);
    return true;
  } else {
    return false;
  }

}

bool MPR121_t::begin(uint8_t address){
  if(address>=0x5A && address<=0x5D) // addresses only valid 0x5A to 0x5D
  {
    this->address = address; // need to be specific here
  }
  return begin();
}

void MPR121_t::run(){
  if(!isInited()) return;
  setRegister(MPR121_ECR, ECR_backup); // restore backup to return to run mode
}

void MPR121_t::stop(){
  if(!isInited()) return;
  ECR_backup = getRegister(MPR121_ECR);	// backup MPR121_ECR to restore when we enter run
  setRegister(MPR121_ECR, ECR_backup & 0xC0); // turn off all electrodes to stop
}

bool MPR121_t::reset(){
  // return true if we successfully reset a device at the
  // address we are expecting

  // MPR121_AFE2 is one of the few registers that defaults to a non-zero value -
  // checking it is sensible as reading back an incorrect value implies
  // something went wrong - we also check MPR121_TS2 bit 7 to see if we have an
  // overcurrent flag set

  setRegister(MPR121_SRST, 0x63); // soft reset

  if(getRegister(MPR121_AFE2)!=0x24){
    error |= 1<<READBACK_FAIL_BIT;
  } else {
    error &= ~(1<<READBACK_FAIL_BIT);
  }

  if((getRegister(MPR121_TS2)&0x80)!=0){
    error |= 1<<OVERCURRENT_FLAG_BIT;
  } else {
    error &= ~(1<<OVERCURRENT_FLAG_BIT);
  }

  if(getError()==NOT_INITED || getError()==NO_ERROR){ // if our only error is that we are not inited...
    return true;
  } else {
    return false;
  }
}

void MPR121_t::applySettings(MPR121_settings_t *settings){
  bool wasRunning = running;
  if(wasRunning) stop();  // can't change most regs when running - checking
  // here avoids multiple stop() / run() calls

  setRegister(MPR121_MHDR,settings->MHDR);
  setRegister(MPR121_NHDR,settings->NHDR);
  setRegister(MPR121_NCLR,settings->NCLR);
  setRegister(MPR121_FDLR,settings->FDLR);
  setRegister(MPR121_MHDF,settings->MHDF);
  setRegister(MPR121_NHDF,settings->NHDF);
  setRegister(MPR121_NCLF,settings->NCLF);
  setRegister(MPR121_FDLF,settings->FDLF);
  setRegister(MPR121_NHDT,settings->NHDT);
  setRegister(MPR121_NCLT,settings->NCLT);
  setRegister(MPR121_FDLT,settings->FDLT);
  setRegister(MPR121_MHDPROXR,settings->MHDPROXR);
  setRegister(MPR121_NHDPROXR,settings->NHDPROXR);
  setRegister(MPR121_NCLPROXR,settings->NCLPROXR);
  setRegister(MPR121_FDLPROXR,settings->FDLPROXR);
  setRegister(MPR121_MHDPROXF,settings->MHDPROXF);
  setRegister(MPR121_NHDPROXF,settings->NHDPROXF);
  setRegister(MPR121_NCLPROXF,settings->NCLPROXF);
  setRegister(MPR121_FDLPROXF,settings->FDLPROXF);
  setRegister(MPR121_NHDPROXT,settings->NHDPROXT);
  setRegister(MPR121_NCLPROXT,settings->NCLPROXT);
  setRegister(MPR121_FDLPROXT,settings->FDLPROXT);
  setRegister(MPR121_DTR, settings->DTR);
  setRegister(MPR121_AFE1, settings->AFE1);
  setRegister(MPR121_AFE2, settings->AFE2);
  setRegister(MPR121_ACCR0, settings->ACCR0);
  setRegister(MPR121_ACCR1, settings->ACCR1);
  setRegister(MPR121_USL, settings->USL);
  setRegister(MPR121_LSL, settings->LSL);
  setRegister(MPR121_TL, settings->TL);

  setRegister(MPR121_ECR, settings->ECR);

  error &= ~(1<<NOT_INITED_BIT); // clear not inited error as we have just inited!
  setTouchThreshold(settings->TTHRESH);
  setReleaseThreshold(settings->RTHRESH);
  setInterruptPin(settings->INTERRUPT);

  if(wasRunning) run();
}

mpr121_error_t MPR121_t::getError(){

  // important - this resets the IRQ pin - as does any I2C comms

  getRegister(MPR121_OORS1);	// OOR registers - we may not have read them yet,
  getRegister(MPR121_OORS2);	// whereas the other errors should have been caught

  // order of error precedence is determined in this logic block

  if(!isInited()) return NOT_INITED; // this has its own checker function

  if((error & (1<<ADDRESS_UNKNOWN_BIT)) != 0){
    return ADDRESS_UNKNOWN;
  } else if((error & (1<<READBACK_FAIL_BIT)) != 0){
    return READBACK_FAIL;
  } else if((error & (1<<OVERCURRENT_FLAG_BIT)) != 0){
    return OVERCURRENT_FLAG;
  } else if((error & (1<<OUT_OF_RANGE_BIT)) != 0){
    return OUT_OF_RANGE;
  } else return NO_ERROR;

}

void MPR121_t::clearError(){
  error = 0;
}

bool MPR121_t::isRunning(){
  return running;
}

bool MPR121_t::isInited(){
  return (error & (1<<NOT_INITED_BIT)) == 0;
}

void MPR121_t::updateTouchData(){
  if(!isInited()) return;

  autoTouchStatusFlag = false;

  lastTouchData = touchData;
  touchData = (unsigned int)getRegister(MPR121_TS1) + ((unsigned int)getRegister(MPR121_TS2)<<8);
}

bool MPR121_t::getTouchData(uint8_t electrode){
  if(electrode>12 || !isInited()) return false; // avoid out of bounds behaviour

  return((touchData>>electrode)&1);
}

uint8_t MPR121_t::getNumTouches(){
  if(!isInited()) return(0xFF);

  uint8_t scratch = 0;
  for(uint8_t i=0; i<13; i++){
    if(getTouchData(i)) scratch++;
  }

  return(scratch);
}

bool MPR121_t::getLastTouchData(uint8_t electrode){
  if(electrode>12 || !isInited()) return false; // avoid out of bounds behaviour

  return((lastTouchData>>electrode)&1);
}

bool MPR121_t::updateFilteredData(){
  if(!isInited()) return(false);

  if(touchStatusChanged()) {
    autoTouchStatusFlag = true;
  }

  for(int i=0; i<13; i++){ // 13 filtered values
    if(touchStatusChanged()) {
      autoTouchStatusFlag = true;
    }
    filteredData[i] = wiringPiI2CReadReg16(fd,MPR121_E0FDL+(i*2));
  }
  return(true);
}

int16_t MPR121_t::getFilteredData(uint8_t electrode){
  if(electrode>12 || !isInited()) return(0xFFFF); // avoid out of bounds behaviour

  return(filteredData[electrode]);
}

bool MPR121_t::updateBaselineData(){
  if(!isInited()) return(false);

  if(touchStatusChanged()) {
    autoTouchStatusFlag = true;
  }

  for(int i=0; i<13; i++){ // 13 filtered values
    if(touchStatusChanged()) {
      autoTouchStatusFlag = true;
    }
    baselineData[i] = wiringPiI2CReadReg8(fd,MPR121_E0BV+i)<<2;
  }
  return(true);

}

int16_t MPR121_t::getBaselineData(uint8_t electrode){
  if(electrode>12 || !isInited()) return(0xFFFF); // avoid out of bounds behaviour

  return(baselineData[electrode]);
}

bool MPR121_t::isNewTouch(uint8_t electrode){
  if(electrode>12 || !isInited()) return(false); // avoid out of bounds behaviour
  return((getLastTouchData(electrode) == false) && (getTouchData(electrode) == true));
}

bool MPR121_t::isNewRelease(uint8_t electrode){
  if(electrode>12 || !isInited()) return(false); // avoid out of bounds behaviour
  return((getLastTouchData(electrode) == true) && (getTouchData(electrode) == false));
}

void MPR121_t::updateAll(){
  updateTouchData();
  updateBaselineData();
  updateFilteredData();
}

void MPR121_t::setTouchThreshold(uint8_t val){
  if(!isInited()) return;
  bool wasRunning = running;

  if(wasRunning) stop();	// can only change thresholds when not running
  // checking here avoids multiple stop() / run()
  // calls

  for(uint8_t i=0; i<13; i++){
    setTouchThreshold(i, val);
  }

  if(wasRunning) run();
}

void MPR121_t::setTouchThreshold(uint8_t electrode, uint8_t val){
  if(electrode>12 || !isInited()) return; // avoid out of bounds behaviour

  // this relies on the internal register map of the MPR121
  setRegister(MPR121_E0TTH + (electrode<<1), val);
}

void MPR121_t::setReleaseThreshold(uint8_t val){
  if(!isInited()) return;
  bool wasRunning = running;

  if(wasRunning) stop();	// can only change thresholds when not running
  // checking here avoids multiple stop / starts

  for(uint8_t i=0; i<13; i++){
    setReleaseThreshold(i,val);
  }

  if(wasRunning) run();
}

void MPR121_t::setReleaseThreshold(uint8_t electrode, uint8_t val){
  if(electrode>12 || !isInited()) return; // avoid out of bounds behaviour

  // this relies on the internal register map of the MPR121
  setRegister(MPR121_E0RTH + (electrode<<1), val);
}

uint8_t MPR121_t::getTouchThreshold(uint8_t electrode){
  if(electrode>12 || !isInited()) return(0xFF); // avoid out of bounds behaviour
  return(getRegister(MPR121_E0TTH+(electrode<<1))); // "255" issue is in here somewhere
}
uint8_t MPR121_t::getReleaseThreshold(uint8_t electrode){
  if(electrode>12 || !isInited()) return(0xFF); // avoid out of bounds behaviour
  return(getRegister(MPR121_E0RTH+(electrode<<1))); // "255" issue is in here somewhere
}

void MPR121_t::setInterruptPin(uint8_t pin){
  // :: here forces the compiler to use Arduino's pinMode, not MPR121's
  if(!isInited()) return;
  ::pinMode(pin, INPUT);
  pullUpDnControl(pin, PUD_UP);
  interruptPin = pin;
}

bool MPR121_t::touchStatusChanged(){
  // :: here forces the compiler to use Arduino's digitalRead, not MPR121's
  return(autoTouchStatusFlag || (!::digitalRead(interruptPin)));
}

void MPR121_t::setProxMode(mpr121_proxmode_t mode){

  if(!isInited()) return;

  bool wasRunning = running;

  if(wasRunning) stop();

  switch(mode){
    case PROX_DISABLED:
      ECR_backup &= ~(3<<4);	// ELEPROX[0:1] = 00
      break;
    case PROX0_1:
      ECR_backup |=  (1<<4);	// ELEPROX[0:1] = 01
      ECR_backup &= ~(1<<5);
      break;
    case PROX0_3:
      ECR_backup &= ~(1<<4);	// ELEPROX[0:1] = 10
      ECR_backup |=  (1<<5);
      break;
    case PROX0_11:
      ECR_backup |=  (3<<4);	// ELEPROX[0:1] = 11
      break;
  }

  if(wasRunning) run();
}

void MPR121_t::setCalibrationLock(mpr121_cal_lock_t lock){

  if(!isInited()) return;

  bool wasRunning = running;

  if(wasRunning) stop();

  switch(lock){
    case CAL_LOCK_ENABLED:
      ECR_backup &= ~(3<<6);  // CL[1:0] = 00
      break;
    case CAL_LOCK_DISABLED:
      ECR_backup |=  (1<<6);  // CL[1:0] = 01
      ECR_backup &= ~(1<<7);      
      break;
    case CAL_LOCK_ENABLED_5_BIT_COPY:
      ECR_backup &= ~(1<<6);  // CL[1:0] = 10
      ECR_backup |=  (1<<7);      
      break;
    case CAL_LOCK_ENABLED_10_BIT_COPY:
      ECR_backup |=  (3<<4);  // CL[1:0] = 11
      break;
  }
  
  if(wasRunning) run();
}


void MPR121_t::setNumDigPins(uint8_t numPins){
  if(!isInited()) return;
  bool wasRunning = running;

  if(numPins>8) numPins = 8; // maximum number of GPIO pins is 8 out of 12

  if(wasRunning){
    stop(); // have to stop to change MPR121_ECR
  }
  ECR_backup = (0x0F&(12-numPins)) | (ECR_backup&0xF0);
  if(wasRunning){
    run();
  }

}

void MPR121_t::setNumEnabledElectrodes(uint8_t numElectrodes){
  if(!isInited()) return;
  bool wasRunning = running;

  if(numElectrodes>12) numElectrodes = 12; // avoid out-of-bounds behaviour
  
  if(wasRunning){
    stop(); // have to stop to change MPR121_ECR
  }
  ECR_backup = (0x0F&numElectrodes) | (ECR_backup&0xF0);
  if(wasRunning){
    run();
  }
}

void MPR121_t::pinMode(uint8_t electrode, int mode){
  if(!isInited()) return;

  if(electrode<4 || electrode >11 || !isInited()) return;

  uint8_t bitmask = 1<<(electrode-4);

  switch (mode) {
    case OUTPUT:
      // MPR121_EN = 1
      // MPR121_DIR = 1
      // MPR121_CTL0 = 0
      // MPR121_CTL1 = 0
      setRegister(MPR121_EN, getRegister(MPR121_EN) | bitmask);
      setRegister(MPR121_DIR, getRegister(MPR121_DIR) | bitmask);
      setRegister(MPR121_CTL0, getRegister(MPR121_CTL0) & ~bitmask);
      setRegister(MPR121_CTL1, getRegister(MPR121_CTL1) & ~bitmask);
      break;
    case INPUT:
      // MPR121_EN = 1
      // MPR121_DIR = 0
      // MPR121_CTL0 = 0
      // MPR121_CTL1 = 0
      setRegister(MPR121_EN, getRegister(MPR121_EN) | bitmask);
      setRegister(MPR121_DIR, getRegister(MPR121_DIR) & ~bitmask);
      setRegister(MPR121_CTL0, getRegister(MPR121_CTL0) & ~bitmask);
      setRegister(MPR121_CTL1, getRegister(MPR121_CTL1) & ~bitmask);
      break;
    case INPUT_PULLUP:
      // MPR121_EN = 1
      // MPR121_DIR = 0
      // MPR121_CTL0 = 1
      // MPR121_CTL1 = 1
      setRegister(MPR121_EN, getRegister(MPR121_EN) | bitmask);
      setRegister(MPR121_DIR, getRegister(MPR121_DIR) & ~bitmask);
      setRegister(MPR121_CTL0, getRegister(MPR121_CTL0) | bitmask);
      setRegister(MPR121_CTL1, getRegister(MPR121_CTL1) | bitmask);
      break;
    case INPUT_PULLDOWN:
      // MPR121_EN = 1
      // MPR121_DIR = 0
      // MPR121_CTL0 = 1
      // MPR121_CTL1 = 0
      setRegister(MPR121_EN, getRegister(MPR121_EN) | bitmask);
      setRegister(MPR121_DIR, getRegister(MPR121_DIR) & ~bitmask);
      setRegister(MPR121_CTL0, getRegister(MPR121_CTL0) | bitmask);
      setRegister(MPR121_CTL1, getRegister(MPR121_CTL1) & ~bitmask);
      break;
    case OUTPUT_HIGHSIDE:
      // MPR121_EN = 1
      // MPR121_DIR = 1
      // MPR121_CTL0 = 1
      // MPR121_CTL1 = 1
      setRegister(MPR121_EN, getRegister(MPR121_EN) | bitmask);
      setRegister(MPR121_DIR, getRegister(MPR121_DIR) | bitmask);
      setRegister(MPR121_CTL0, getRegister(MPR121_CTL0) | bitmask);
      setRegister(MPR121_CTL1, getRegister(MPR121_CTL1) | bitmask);
      break;      
    case OUTPUT_LOWSIDE:
      // MPR121_EN = 1
      // MPR121_DIR = 1
      // MPR121_CTL0 = 1
      // MPR121_CTL1 = 0
      setRegister(MPR121_EN, getRegister(MPR121_EN) | bitmask);
      setRegister(MPR121_DIR, getRegister(MPR121_DIR) | bitmask);
      setRegister(MPR121_CTL0, getRegister(MPR121_CTL0) | bitmask);
      setRegister(MPR121_CTL1, getRegister(MPR121_CTL1) & ~bitmask);      
      break;
    default:
      break;
  }
}

void MPR121_t::digitalWrite(uint8_t electrode, uint8_t val){

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !isInited()) return;

  if(val){
    setRegister(MPR121_SET, 1<<(electrode-4));
  } else {
    setRegister(MPR121_CLR, 1<<(electrode-4));
  }
}

void MPR121_t::digitalToggle(uint8_t electrode){

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !isInited()) return;

  setRegister(MPR121_TOG, 1<<(electrode-4));
}

bool MPR121_t::digitalRead(uint8_t electrode){

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !isInited()) return false;

  return(((getRegister(MPR121_DAT)>>(electrode-4))&1)==1);
}

void MPR121_t::analogWrite(uint8_t electrode, uint8_t value){
  // LED output 5 (ELE9) and output 6 (ELE10) have a PWM bug
  // https://community.nxp.com/thread/305474

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !isInited()) return;

  uint8_t shiftedVal = value>>4;

  if(shiftedVal > 0){
    setRegister(MPR121_SET, 1<<(electrode-4)); // normal PWM operation
  } else {
    // this make a 0 PWM setting turn off the output
    setRegister(MPR121_CLR, 1<<(electrode-4));
  }

  uint8_t scratch;

  switch(electrode-4){

    case 0:
      scratch = getRegister(MPR121_PWM0);
      setRegister(MPR121_PWM0, (shiftedVal & 0x0F) | (scratch & 0xF0));
      break;
    case 1:
      scratch = getRegister(MPR121_PWM0);
      setRegister(MPR121_PWM0, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
      break;
    case 2:
      scratch = getRegister(MPR121_PWM1);
      setRegister(MPR121_PWM1, (shiftedVal & 0x0F) | (scratch & 0xF0));
      break;
    case 3:
      scratch = getRegister(MPR121_PWM1);
      setRegister(MPR121_PWM1, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
      break;
    case 4:
      scratch = getRegister(MPR121_PWM2);
      setRegister(MPR121_PWM2, (shiftedVal & 0x0F) | (scratch & 0xF0));
      break;
    case 5:
      scratch = getRegister(MPR121_PWM2);
      setRegister(MPR121_PWM2, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
      break;
    case 6:
      scratch = getRegister(MPR121_PWM3);
      setRegister(MPR121_PWM3, (shiftedVal & 0x0F) | (scratch & 0xF0));
      break;
    case 7:
      scratch = getRegister(MPR121_PWM3);
      setRegister(MPR121_PWM3, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
      break;
  }
}

void MPR121_t::setSamplePeriod(mpr121_sample_interval_t period){

  uint8_t scratch;

  scratch = getRegister(MPR121_AFE2);

#if DEBUG
  printf("setSamplePeriod before: %d", scratch);
#endif

  setRegister(MPR121_AFE2, (scratch & 0xF8) | (period & 0x07));

#if DEBUG
  printf("setSamplePeriod after: %d", getRegister(MPR121_AFE2));
#endif
}

void MPR121_t::enableRepeatedStarts(){

  // only required for pre-May 2017 Raspbian - stat test checks 
  // for the appropriate device file before getting deep into things

  // modifies /sys/module/i2c_bcm2708/parameters/combined (if it exists) 
  // to contain "Y" (strangely by writing a 1 to it) - but only if it 
  // currently contains an "N"

  // this enables repeated start behaviour on the I2C bus, which the MPR121
  // requires

  char path[] = "/sys/module/i2c_bcm2708/parameters/combined";
  struct stat stat_buffer;   

  if(stat (path, &stat_buffer) != 0){
    // if the device file doesn't exist, we're using the newer I2C driver
    // and don't need to explicitly enable repeated starts
    return; 
  }

  FILE *fp;
  char read_buffer[8]; // way bigger than required, but avoids overflow
  char command[128]; // way bigger than required, but avoids overflow
  // this is a shell one-liner to output a 1 (which becomes a Y) to the device file
  sprintf(command, "%s%s%s", "echo -n 1 | sudo tee ", path, " > /dev/null");

  // open the device file
  fp = fopen(path, "r");
  if (fp == NULL) {
    printf("%s%s%s\n","Failed to open ", path, " - communication with MPR121 may not work" );
  } else {
    // if we can read it, pull it into a buffer
    fgets(read_buffer, sizeof(read_buffer)-1, fp);
    fclose(fp);
    if (strstr(read_buffer, "N") != NULL) {
      // if we find an N, we should change it to a Y to enable repeated starts
      system(command);
    }
  }

}

MPR121_t MPR121 = MPR121_t();
