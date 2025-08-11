cmake_minimum_required(VERSION 3.13)
project(RtcPs2BusInterface C)

set(MCU atmega88pa)
# set(MCU atmega328p)
set(F_CPU 8000000UL)
set(BAUD 9600)

set(PROJECT_BINARY ${CMAKE_PROJECT_NAME}.elf)
set(PROJECT_HEX ${CMAKE_PROJECT_NAME}.hex)
set(PROJECT_EEP ${CMAKE_PROJECT_NAME}.eep)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_C_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU} -Os -std=gnu11")

add_executable(${PROJECT_BINARY} src/controller.c)

add_custom_command(TARGET ${PROJECT_BINARY} POST_BUILD
    COMMAND avr-objcopy -O ihex -R .eeprom ${PROJECT_BINARY} ${PROJECT_HEX}
    COMMAND avr-objcopy -O ihex -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 ${PROJECT_BINARY} ${PROJECT_EEP}
    COMMAND avr-size -C --mcu=${MCU} ${PROJECT_BINARY}
    COMMENT "Memory usage (Flash, RAM, EEPROM):"
)

if(MCU STREQUAL "atmega88" OR MCU STREQUAL "atmega88pa")
    set(FUSE_L 0xE2)
    set(FUSE_H 0xDF)
    set(FUSE_E 0xFF)
    set(MINIPRO_PART ATMEGA88)
elseif(MCU STREQUAL "atmega328p")
    set(FUSE_L 0xE2)
    set(FUSE_H 0x77)
    set(FUSE_E 0xFF)
    set(MINIPRO_PART ATMEGA328P)
else()
    message(WARNING "Unknown MCU: ${MCU}. Fuse values not set.")
endif()

add_custom_target(fuses
    COMMAND minipro -p ${MINIPRO_PART} -w /dev/null -c config -e -P
            -fuseL ${FUSE_L} -fuseH ${FUSE_H} -fuseE ${FUSE_E}
    COMMENT "Flashing fuses with minipro (TL866II Plus) for ${MCU}"
)

# Flash firmware and EEPROM using minipro
add_custom_target(flash
  COMMAND minipro -p ${MINIPRO_PART} -w ${PROJECT_HEX}
  COMMAND minipro -p ${MINIPRO_PART} -w ${PROJECT_EEP}
  COMMENT "Flashing firmware and EEPROM to ${MCU} using minipro (TL866II Plus)"
)