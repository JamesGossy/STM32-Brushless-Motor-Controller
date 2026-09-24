cmake_minimum_required(VERSION 3.20)
project(foc_g474 C)
set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(APP_SRC
  app/app.c
  app/calib.c
  app/can_proto.c
  app/cfg.c
  app/cmd.c
  app/foc.c
  app/loops.c
  app/svpwm.c
  app/telem.c
  app/trig.c
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set(WARN -Wall -Wextra -Wno-unused-parameter)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Generic")
  # ---- STM32G474 firmware ----
  enable_language(ASM)
  set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
  set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
  file(GLOB TARGET_SRC CONFIGURE_DEPENDS hal/stm32g474/*.c)

  add_executable(${PROJECT_NAME}.elf ${APP_SRC} ${TARGET_SRC} hal/stm32g474/startup_stm32g474retx.s)
  target_include_directories(${PROJECT_NAME}.elf PRIVATE app hal hal/stm32g474 hal/stm32g474/cmsis)
  target_compile_definitions(${PROJECT_NAME}.elf PRIVATE STM32G474xx)
  target_compile_options(${PROJECT_NAME}.elf PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${WARN} -fsingle-precision-constant -fno-math-errno>)
  target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/hal/stm32g474/STM32G474RE.ld
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,-u,_printf_float
  )
  target_link_libraries(${PROJECT_NAME}.elf PRIVATE m)

  add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
  )

  set(PROGRAMMER STM32_Programmer_CLI CACHE STRING "STM32CubeProgrammer CLI")
  add_custom_target(flash
    COMMAND ${PROGRAMMER} -c port=SWD mode=UR -w ${PROJECT_NAME}.elf -v -rst
    DEPENDS ${PROJECT_NAME}.elf
    USES_TERMINAL
  )
else()
  # ---- host: software-in-the-loop simulator and tests ----
  add_library(foc_sil STATIC ${APP_SRC} sim/sim.c sim/motor_model.c)
  target_include_directories(foc_sil PUBLIC app hal sim)
  target_compile_options(foc_sil PUBLIC ${WARN})
  if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(foc_sil PUBLIC -fsingle-precision-constant)
  endif()
  if(MSVC)
    target_compile_definitions(foc_sil PUBLIC _CRT_SECURE_NO_WARNINGS)
  else()
    target_link_libraries(foc_sil PUBLIC m)
  endif()

  add_executable(foc_sim sim/sim_main.c sim/net.c)
  target_link_libraries(foc_sim PRIVATE foc_sil)
  if(WIN32)
    target_link_libraries(foc_sim PRIVATE ws2_32)
  endif()

  enable_testing()
  file(GLOB UNIT_TESTS CONFIGURE_DEPENDS tests/unit/test_*.c)
  foreach(src ${UNIT_TESTS})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_include_directories(${name} PRIVATE tests)
    target_link_libraries(${name} PRIVATE foc_sil)
    add_test(NAME unit_${name} COMMAND ${name})
  endforeach()

  add_executable(test_sil tests/integration/test_sil.c)
  target_include_directories(test_sil PRIVATE tests)
  target_link_libraries(test_sil PRIVATE foc_sil)
  set(SIL_CASES
    boot calibration not_calibrated torque_step speed_tracking load_rejection
    field_weakening reverse can_control can_timeout serial_latched estop
    overcurrent current_sensor_fault undervoltage overtemp encoder_fault encoder_glitch stall gate_fault telemetry)
  foreach(c ${SIL_CASES})
    add_test(NAME sil_${c} COMMAND test_sil ${c})
  endforeach()
endif()
