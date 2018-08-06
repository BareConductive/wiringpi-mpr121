#!/usr/bin/env python

################################################################################
#
# Bare Conductive Pi Cap
# ----------------------
#
# setup.py - install the Python bindings for the MPR121 library.
# Invoke with "sudo python setup.py install" or "sudo python3 setup.py install"
#
# Written for Raspberry Pi.
#
# Bare Conductive code written by Tom Hartley.
#
# This work is licensed under a MIT license https://opensource.org/licenses/MIT
#
# Copyright (c) 2018, Bare Conductive
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#################################################################################

from distutils.core import setup, Extension


mpr121_module = Extension('_MPR121',
                           sources=['MPR121.i'],
                           include_dirs = ['../src/'],
#                          library_dirs = ['../src/'],
                           libraries = ['MPR121','wiringPi'],
                           swig_opts = ['-c++'])

setup (name = 'MPR121',
       version = '1.0',
       author      = "Bare Conductive",
       description = """Library for interfacing with the MPR121 through Python""",
       ext_modules = [mpr121_module],
       package_dir={'MPR121': ''},
       packages=['MPR121'],
       )
