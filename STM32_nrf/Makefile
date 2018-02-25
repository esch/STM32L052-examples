##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = stm32_nrf
OBJS = ./tinysh/tinysh.o ./tinysh/tinysh_setup.o ./nRF/nRF24L01P_Maniacbug.o   
OBJS += ./EP/epdif.o ./EP/epd1in54.o ./EP/epdpaint.o ./EP/epd_app.o  ./EP/font8.o ./EP/font12.o ./EP/font16.o ./EP/font20.o  ./EP/font24.o ./imagedata.o
LDSCRIPT = ../stm32l0xx8.ld
DEFS = -I./tinysh/ -I./lmic/ -I./hal/ -I./nRF -I. -DCFG_eu868 -DCFG_sx1276_radio
DEFS += -I./EP/
SCRIPT_DIR=..

default:  $(OBJS) $(BINARY).bin

include ../Makefile.include  
