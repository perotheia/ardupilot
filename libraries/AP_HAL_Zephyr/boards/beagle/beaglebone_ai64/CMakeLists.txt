# BeagleBone AI-64 (TI J721E), ArduPilot on the MAIN R5FSS1 core0 (lockstep)
# beside Linux + TI vision_apps; pulled in from app/CMakeLists.txt.

# Must match what Zephyr resolves for the J721E R5F (target_arm.cmake:
# cortex-r5, VFP_DP_D16 -> vfpv3-d16, hard float) or the WAF objects will
# not link against the kernel.
set(AP_BOARD_ABI_CFLAGS
    -mcpu=cortex-r5
    -mthumb
    -mfpu=vfpv3-d16
    -mfloat-abi=hard
    -mfp16-format=ieee
    PARENT_SCOPE)

# RPMsg to Linux (k3_r5_remoteproc + rpmsg_tty): SERIAL0 = MAVLink2.
if(CONFIG_OPENAMP)
    zephyr_library_sources(${CMAKE_CURRENT_LIST_DIR}/../../../ipc/rpmsg_remote.c)
    zephyr_library_sources(${CMAKE_CURRENT_LIST_DIR}/ai64_board.c)
endif()
