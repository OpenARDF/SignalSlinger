SHELL := cmd.exe

PROJECT := SignalSlingerBootloader
CONFIGURATION := Release
TARGET := $(PROJECT).elf

TOOLCHAIN_ROOT ?= C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain
PACK_ROOT ?= C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\AVR-Dx_DFP\1.10.114

AVR_GPP := "$(TOOLCHAIN_ROOT)\bin\avr-g++.exe"
AVR_OBJCOPY := "$(TOOLCHAIN_ROOT)\bin\avr-objcopy.exe"
AVR_SIZE := "$(TOOLCHAIN_ROOT)\bin\avr-size.exe"

MCU := avr128da28
DEVICE_PATH := "$(PACK_ROOT)\gcc\dev\$(MCU)"
PACK_INCLUDE := "$(PACK_ROOT)\include"

ROOT := ..
APP_ROOT := ..\..\SignalSlinger

CXXFLAGS := -funsigned-char -funsigned-bitfields -DNDEBUG -DF_CPU=24000000UL
CXXFLAGS += -I"$(ROOT)\include" -I"$(APP_ROOT)\include" -I"$(APP_ROOT)\utils" -I"$(APP_ROOT)\utils\assembler" -I"$(APP_ROOT)\Config" -I$(PACK_INCLUDE)
CXXFLAGS += -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -mmcu=$(MCU) -B $(DEVICE_PATH)
CXXFLAGS += -fno-threadsafe-statics -MD -MP

ASFLAGS := -x assembler-with-cpp -mmcu=$(MCU) -B $(DEVICE_PATH) -I"$(APP_ROOT)\utils" -I$(PACK_INCLUDE)

LDFLAGS := -Wl,-Map="$(PROJECT).map" -Wl,--gc-sections -mmcu=$(MCU) -B $(DEVICE_PATH)

OBJECTS := src\main.o src\protected_io.o

.PHONY: all clean

all: $(TARGET) $(PROJECT).hex $(PROJECT).lss
	$(AVR_SIZE) --mcu=$(MCU) -C $(TARGET)

src\main.o: ..\src\main.cpp ..\include\bootloader_config.h
	if not exist src mkdir src
	$(AVR_GPP) $(CXXFLAGS) -c -o $@ $<

src\protected_io.o: ..\..\SignalSlinger\src\protected_io.S
	if not exist src mkdir src
	$(AVR_GPP) $(ASFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	$(AVR_GPP) -o $(TARGET) $(OBJECTS) $(LDFLAGS)

$(PROJECT).hex: $(TARGET)
	$(AVR_OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures $(TARGET) $(PROJECT).hex

$(PROJECT).lss: $(TARGET)
	"$(TOOLCHAIN_ROOT)\bin\avr-objdump.exe" -h -S $(TARGET) > $(PROJECT).lss

clean:
	if exist src rmdir /S /Q src
	if exist $(TARGET) del /Q $(TARGET)
	if exist $(PROJECT).hex del /Q $(PROJECT).hex
	if exist $(PROJECT).lss del /Q $(PROJECT).lss
	if exist $(PROJECT).map del /Q $(PROJECT).map
