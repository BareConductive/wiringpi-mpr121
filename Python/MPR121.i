%module MPR121

%include "stdint.i"
/* rename functions into snake_case */
%feature("autodoc", "1");
%rename(get_error) getError;
%rename(clear_error) clearError;
%rename(touch_status_changed) touchStatusChanged;
%rename(update_touch_data) updateTouchData;
%rename(update_baseline_data) updateBaselineData;
%rename(update_filtered_data) updateFilteredData;
%rename(update_all) updateAll;
%rename(get_touch_data) getTouchData;
%rename(get_num_touches) getNumTouches;
%rename(get_filtered_data) getFilteredData;
%rename(get_baseline_data) getBaselineData;
%rename(is_new_touch) isNewTouch;
%rename(is_new_release) isNewRelease;
%rename(set_touch_threshold) setTouchThreshold;
%rename(set_release_threshold) setReleaseThreshold;
%rename(get_touch_threshold) getTouchThreshold;
%rename(apply_settings) applySettings;
%rename(set_register) setRegister;
%rename(get_register) getRegister;
%rename(is_running) isRunning;
%rename(is_inited) isInited;
%rename(set_interrupt_pin) setInterruptPin;
%rename(set_prox_mode) setProxMode;
%rename(set_num_dig_pins) setNumDigPins;
%rename(pin_mode) pinMode;
%rename(digital_write) digitalWrite;
%rename(digital_toggle) digitalToggle;
%rename(digital_read) digitalRead;
%rename(analog_write) analogWrite;
%rename(set_calibration_lock) setCalibrationLock;
%rename(_set_sample_period) setSamplePeriod; /* internal, we overwrite this ourselves to work around CPP enums */

%{
  #include "../src/MPR121.h"
  #include "../src/MPR121_defs.h"
  #include "/usr/include/wiringPi.h"
  #include "/usr/include/wiringPiI2C.h"
%}

/* let's just grab the original header file here */
%include "../src/MPR121.h"

/* error struct */
%inline %{
  struct MPR121Error {
    enum { NO_ERROR, RETURN_TO_SENDER, ADDRESS_UNKNOWN, READBACK_FAIL, OVERCURRENT_FLAG, OUT_OF_RANGE, NOT_INITED };
  };
%}

/* sample interval struct */
%inline %{
  struct MPR121SampleInterval {
    enum {
      SAMPLE_INTERVAL_1MS   = 0x00,
      SAMPLE_INTERVAL_2MS   = 0x01,
      SAMPLE_INTERVAL_4MS   = 0x02,
      SAMPLE_INTERVAL_8MS   = 0x03,
      SAMPLE_INTERVAL_16MS  = 0x04,
      SAMPLE_INTERVAL_32MS  = 0x05,
      SAMPLE_INTERVAL_64MS  = 0x06,
      SAMPLE_INTERVAL_128MS = 0x07
    };
  };
%}
