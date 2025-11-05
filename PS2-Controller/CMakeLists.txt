cmake_minimum_required(VERSION 3.13)
project(RtcPs2BusInterface C)

set(MCU atmega88)
# set(MCU atmega328p)
set(F_CPU 8000000UL)
set(BAUD 9600)

set(PROJECT_BINARY ${CMAKE_PROJECT_NAME}.elf)
set(PROJECT_HEX ${CMAKE_PROJECT_NAME}.hex)
set(PROJECT_SREC ${CMAKE_PROJECT_NAME}.srec)
set(PROJECT_EEP ${CMAKE_PROJECT_NAME}.eep)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_C_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU} -Os -std=c11")

add_executable(${PROJECT_BINARY} src/controller.c)

add_custom_command(TARGET ${PROJECT_BINARY} POST_BUILD
    COMMAND avr-objcopy -O srec ${PROJECT_BINARY} ${PROJECT_SREC}
    COMMAND avr-objcopy -O ihex ${PROJECT_BINARY} ${PROJECT_HEX}
    COMMAND avr-size -C --mcu=${MCU} ${PROJECT_BINARY}
    COMMENT "Memory usage (Flash, RAM, EEPROM):"
)

if(MCU STREQUAL "atmega88" OR MCU STREQUAL "atmega88pa")
    set(FUSE_L 0xE2)
    set(FUSE_H 0xFF) # Turn of Reset pin, requires HV programming after this
    set(FUSE_E 0xFF)
    set(MINIPRO_PART ATMEGA88@DIP28)
elseif(MCU STREQUAL "atmega328p")
    set(FUSE_L 0xD2)
    set(FUSE_H 0xD7)  # RSTDISBL=1: Reset pin enabled (normal operation)
    set(FUSE_E 0xFF)
    set(MINIPRO_PART ATMEGA328P@DIP28)
else()
    message(WARNING "Unknown MCU: ${MCU}. Fuse values not set.")
endif()

add_custom_target(fuses
    COMMAND minipro -p ${MINIPRO_PART} -w ../fuses.conf -c config
    COMMENT "Flashing fuses with minipro (TL866II Plus) for ${MCU}"
)

# Flash firmware using minipro
add_custom_target(flash
  COMMAND minipro -p ${MINIPRO_PART} -w ${PROJECT_HEX} -f ihex
  COMMENT "Flashing firmware to ${MCU} using minipro (TL866II Plus)"
)