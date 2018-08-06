from .MPR121 import MPR121_t, MPR121Error, MPR121SampleInterval
from types import MethodType
from math import ceil, log, pow

# simple wrapper over C++ MPT121_t so we can easily:
#   import MPR121
#   sensor = MPR121.begin()

# overwrite for setSamplePeriod so user can set arbitrary value instead of relying on struct from C++
def _set_sample_period(self, target_sample_period):
  # find nearest power of two for given input (13 -> 16, 3 -> 4, ...)
  sample_period = pow(2, ceil(log(target_sample_period) / log(2)))

  # dictionary storing all possible states for internal CPP function
  sample_periods = {
      1:   MPR121SampleInterval.SAMPLE_INTERVAL_1MS,
      2:   MPR121SampleInterval.SAMPLE_INTERVAL_2MS,
      4:   MPR121SampleInterval.SAMPLE_INTERVAL_4MS,
      8:   MPR121SampleInterval.SAMPLE_INTERVAL_8MS,
      16:  MPR121SampleInterval.SAMPLE_INTERVAL_16MS,
      32:  MPR121SampleInterval.SAMPLE_INTERVAL_32MS,
      64:  MPR121SampleInterval.SAMPLE_INTERVAL_64MS,
      128: MPR121SampleInterval.SAMPLE_INTERVAL_128MS
  }

  # set internal CPP function value based on user input,
  # only if we can do this
  if sample_period in sample_periods:
    selected_period = sample_periods[sample_period]
    self._set_sample_period(selected_period)

def begin(address = 0x5C):
  mpr121 = MPR121_t()

  if mpr121.begin(address):
    # add method to existing object with our overwriten sample period setting function
    mpr121.set_sample_period = MethodType(_set_sample_period, mpr121)
    return mpr121
  else:
    errors = {
      MPR121Error.NO_ERROR:         'no error',
      MPR121Error.ADDRESS_UNKNOWN:  'incorrect address',
      MPR121Error.READBACK_FAIL:    'readback failure',
      MPR121Error.OVERCURRENT_FLAG: 'overcurrent on REXT pin',
      MPR121Error.OUT_OF_RANGE:     'electrode out of range',
      MPR121Error.NOT_INITED:       'not initialised'
    }

    error_code = mpr121.get_error()
    error_text = errors.get(error_code, 'unknown error')

    raise Exception('error setting up MPR121: ' + error_text)
