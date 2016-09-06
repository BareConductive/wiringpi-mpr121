#*******************************************************************************
#
# Bare Conductive MPR121 library Python wrapper
# ---------------------------------------------
#
# Makefile - defines build and install recipes for MPR121 Python wrapper
#
# Heavily based on Makefiles from WiringPi written by Gordon Henderson
# (lots of copypasta)
#
# Bare Conductive code written by Stefan Dzisiewski-Smith and Szymon Kaliski.
#
# This work is licensed under a Creative Commons Attribution-ShareAlike 3.0
# Unported License (CC BY-SA 3.0) http://creativecommons.org/licenses/by-sa/3.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#*******************************************************************************

NAME      = MPR121
WRAP      = $(NAME)_wrap.cxx
SO        = _$(NAME).so
PY        = $(NAME).py
IFACE     = $(NAME).i

CC        = g++
INCLUDE   = -I/usr/include/python2.7
CFLAGS    = -fPIC -Wall -Wextra -shared $(WRAP) -L. -L../src/ -lMPR121 -lwiringPi -lpython2.7 $(INCLUDE) -o $(SO)
SWIG      = swig
SWIGFLAGS = -Wall -c++ -python $(IFACE)

all: $(SO)

$(SO): $(PY)
	$(CC) $(CFLAGS)

$(PY): $(IFACE)
	$(SWIG) $(SWIGFLAGS)

.PHONY:	clean
clean:
	rm -f $(WRAP) $(SO) $(PY)
