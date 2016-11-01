#*******************************************************************************
#
# Bare Conductive MPR121 library
# ------------------------------
#
# Makefile - defines build and install recipes for libMPR121.so
#
# Heavily based on Makefiles from WiringPi written by Gordon Henderson
# (lots of copypasta)
#
# Bare Conductive code written by Stefan Dzisiewski-Smith.
#
# This work is licensed under a MIT license https://opensource.org/licenses/MIT
# 
# Copyright (c) 2016, Bare Conductive
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
#*******************************************************************************

VERSION=0.1
DESTDIR?=/usr
PREFIX?=/local

LDCONFIG?=ldconfig

ifneq ($V,1)
	Q ?= @
endif

STATIC=libMPR121.a
DYNAMIC=libMPR121.so.$(VERSION)

#DEBUG	= -g -O0
DEBUG	= -O2
CC	= g++
INCLUDE	= -I.
DEFS	= -D_GNU_SOURCE
CFLAGS	= $(DEBUG) $(DEFS) -Wformat=2 -Wall -Winline $(INCLUDE) -pipe -fPIC

LIBS    =

###############################################################################

SRC	=	MPR121.cpp

HEADERS =	MPR121.h MPR121_defs.h


OBJ	=	$(SRC:.cpp=.o)

all:		$(DYNAMIC)

static:		$(STATIC)

$(STATIC):	$(OBJ)
	$Q echo "[Link (Static)]"
	$Q ar rcs $(STATIC) $(OBJ)
	$Q ranlib $(STATIC)
	#	@size   $(STATIC)

$(DYNAMIC):	$(OBJ)
	$Q echo "[Link (Dynamic)]"
	$Q $(CC) -shared -Wl,-soname,libMPR121.so -o libMPR121.so.$(VERSION) -lpthread $(OBJ)
	$Q ln -sf libMPR121.so.$(VERSION)	libMPR121.so

.c.o:
	$Q echo [Compile] $<
	$Q $(CC) -c $(CFLAGS) $< -o $@


.PHONY:	clean
clean:
	$Q echo "[Clean]"
	$Q rm -f $(OBJ) $(OBJ_I2C) *~ core tags Makefile.bak libMPR121.*

.PHONY:	tags
tags:	$(SRC)
	$Q echo [ctags]
	$Q ctags $(SRC)


.PHONY:	install
install:	$(DYNAMIC)
	$Q echo "[Install Headers]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/include
	$Q install -m 0644 $(HEADERS)					$(DESTDIR)$(PREFIX)/include
	$Q echo "[Install Dynamic Lib]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/lib
	$Q install -m 0755 libMPR121.so.$(VERSION)			$(DESTDIR)$(PREFIX)/lib/libMPR121.so.$(VERSION)
	$Q ln -sf $(DESTDIR)$(PREFIX)/lib/libMPR121.so.$(VERSION)	$(DESTDIR)/lib/libMPR121.so
	$Q $(LDCONFIG)

.PHONY:	install-static
install-static:	$(STATIC)
	$Q echo "[Install Headers]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/include
	$Q install -m 0644 $(HEADERS)					$(DESTDIR)$(PREFIX)/include
	$Q echo "[Install Static Lib]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/lib
	$Q install -m 0755 libMPR121.a					$(DESTDIR)$(PREFIX)/lib

.PHONY:	install-deb
install-deb:	$(DYNAMIC)
	$Q echo "[Install Headers: deb]"
	$Q install -m 0755 -d							~/MPR121/debian-template/MPR121/usr/include
	$Q install -m 0644 $(HEADERS)						~/MPR121/debian-template/MPR121/usr/include
	$Q echo "[Install Dynamic Lib: deb]"
	install -m 0755 -d							~/MPR121/debian-template/MPR121/usr/lib
	install -m 0755 libMPR121.so.$(VERSION)				~/MPR121/debian-template/MPR121/usr/lib/libMPR121.so.$(VERSION)
	ln -sf ~/MPR121/debian-template/MPR121/usr/lib/libMPR121.so.$(VERSION)	~/MPR121/debian-template/MPR121/usr/lib/libMPR121.so

.PHONY:	uninstall
uninstall:
	$Q echo "[UnInstall]"
	$Q cd $(DESTDIR)$(PREFIX)/include/ && rm -f $(HEADERS)
	$Q cd $(DESTDIR)$(PREFIX)/lib/     && rm -f libMPR121.*
	$Q $(LDCONFIG)


.PHONY:	depend
depend:
	makedepend -Y $(SRC) $(SRC_I2C)

# DO NOT DELETE

MPR121.o: MPR121_defs.h MPR121.h

