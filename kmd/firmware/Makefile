# Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ROOT := $(TOP_KMD)
DLA_CONFIG ?= 

TOOLCHAIN_PREFIX ?=

ifeq ($(TOOLCHAIN_PREFIX),)
$(error Toolchain prefix missing)
endif

ifeq ($(NVDLA_CONFIG),)
$(error NVDLA_CONFIG missing, please specify NVDLA_CONFIG=nv_small or nv_large)
endif

$(info NOTICE!! NVDLA_CONFIG set to $(NVDLA_CONFIG))
ifeq ($(NVDLA_CONFIG),nv_small)
DLA_CONFIG := DLA_SMALL_CONFIG
endif

ifeq ($(NVDLA_CONFIG),nv_large)
DLA_CONFIG := DLA_LARGE_CONFIG
endif

MODULE := libnvdla_firmware

include $(ROOT)/make/macros.mk

BUILDOUT ?= $(ROOT)/out/firmware
BUILDDIR := $(BUILDOUT)/$(MODULE)
LIB := $(BUILDDIR)/$(MODULE).so

INCLUDES :=
MODULE_OPTFLAGS ?= -O3
MODULE_COMPILEFLAGS := -fPIC -finline -W -Wall -Wno-multichar -Wno-unused-parameter -Wno-unused-function -Werror-implicit-function-declaration
MODULE_CFLAGS := --std=c99
MODULE_CPPFLAGS := --std=c++11 -fexceptions -fno-rtti

all:: $(LIB)

include rules.mk

# the logic to compile and link stuff is in here
$(LIB): $(ALLMODULE_OBJS)
	@echo building $(MODULE)  $@
	$(TOOLCHAIN_PREFIX)g++ -shared $(ALLMODULE_OBJS) -o $@
