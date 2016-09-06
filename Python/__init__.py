from MPR121 import MPR121_t, MPR121Error

# simple wrapper over C++ MPT121_t so we can easily:
#   import MPR121
#   sensor = MPR121.begin()

def begin(address = 0x5C):
  mpr121 = MPR121_t()

  if mpr121.begin(address):
    return mpr121
  else:
    errors = {
        MPR121Error.NO_ERROR: 'no error',
        MPR121Error.ADDRESS_UNKNOWN: 'incorrect address',
        MPR121Error.READBACK_FAIL: 'readback failure',
        MPR121Error.OVERCURRENT_FLAG: 'overcurrent on REXT pin',
        MPR121Error.OUT_OF_RANGE: 'electrode out of range',
        MPR121Error.NOT_INITED: 'not initialised'
    }

    error_code = mpr121.get_error()
    error_text = errors.get(error_code, 'unknown error')

    raise Exception('error setting up MPR121: ' + error_text)
