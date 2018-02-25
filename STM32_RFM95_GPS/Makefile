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

BINARY = stm32_rfm95_gps
OBJS = ./tinysh/tinysh.o  ./hal/hal.o ./lmic/radio.o ./lmic/aes.o ./lmic/lmic.o ./lmic/oslmic.o ./gps/gps.o
LDSCRIPT = ../stm32l0xx8.ld
DEFS = -I./tinysh/ -I./lmic/ -I./hal/ -I./gps/ -I. -DCFG_eu868 -DCFG_sx1276_radio
SCRIPT_DIR=..

default:  $(OBJS) $(BINARY).bin

include ../Makefile.include  


