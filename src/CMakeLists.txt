include_directories(
	${PROJECT_SOURCE_DIR}/GD32C10x/CMSIS
	${PROJECT_SOURCE_DIR}/GD32C10x/Peripherals/inc
	${PROJECT_SOURCE_DIR}/inc
)

set(ELF_FILE ${PROJECT_NAME}.elf)
set(BIN_FILE ${PROJECT_NAME}.bin)
set(HEX_FILE ${PROJECT_NAME}.hex)

file(GLOB SRC_LIST ${PROJECT_SOURCE_DIR}/src/*.c)
#aux_source_directory(. SRC_LIST)

add_executable(${ELF_FILE} ${SRC_LIST})
target_link_libraries(${ELF_FILE} gd32c10x)

add_custom_command(TARGET "${ELF_FILE}" POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
	COMMAND ${CMAKE_OBJCOPY} -Oihex ${ELF_FILE} ${HEX_FILE}
	COMMAND ${CMAKE_SIZE} ${ELF_FILE} ${HEX_FILE}
	COMMAND ${CMAKE_COMMAND} -E copy ${ELF_FILE} ../${ELF_FILE}
	COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} ../${BIN_FILE}
	COMMAND ${CMAKE_COMMAND} -E copy ${HEX_FILE} ../${HEX_FILE}
)