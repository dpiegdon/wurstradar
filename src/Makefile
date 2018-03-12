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

BINARY = wurstradar
OBJS = 

# may be overriden by user:
DOPPLER_HZ_PER_POINT1_KPH ?= 444
MEASUREMENT_DEGREE ?= 0

OPENCM3_DIR=../libopencm3
CMSIS_DIR=../CMSIS_5
DEVICE=STM32F415RG
DEFS += -I../CMSIS_5/CMSIS/Core/Include/
DEFS += -I../CMSIS_5/CMSIS/DSP/Include/
DEFS += -DARM_MATH_CM4
DEFS += -D__FPU_PRESENT=1
DEFS += -DVERSION=$(VERSION)
DEFS += -DGIT_VERSION=$(GIT_VERSION)
DEFS += -DDOPPLER_HZ_PER_POINT1_KPH=$(DOPPLER_HZ_PER_POINT1_KPH)
DEFS += -DMEASUREMENT_DEGREE=$(MEASUREMENT_DEGREE)

VERSION := 1.0.0
GITHASH := $(shell git describe --always)
GITDIRTY := $(shell git status -s | wc -c | sed -e 's/^[1-9][0-9]*$$/-DIRTY/' -e 's/^0$$//')
GIT_VERSION := $(GITHASH)$(GITDIRTY)

include ../libopencm3.target.mk

