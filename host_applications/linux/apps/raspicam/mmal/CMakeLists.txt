
cmake_minimum_required(VERSION 3.0)

########################################
##### set to your path to userland #####
set(USER_LAND /home/pi/work/userland)
########################################

include_directories(
  ${USER_LAND}
  ${USER_LAND}/host_applications/linux/libs/bcm_host/include
  ${USER_LAND}/host_applications/linux/apps/raspicam
  ${USER_LAND}/host_applications/linux/libs/sm
  )

set(COMMON_SOURCES
  RaspiCamControl.c
  RaspiCLI.c
  RaspiPreview.c
  RaspiCommonSettings.c
  RaspiHelpers.c
  )

set(LIB_SOURCE ${PROJECT_SOURCE_DIR}/MmalEncoder.c)

set(BIN_SOURCE ${PROJECT_SOURCE_DIR}/MmalMain.c)

# make shared liberary libMmalEncoder.so
add_library(MmalEncoder SHARED
  ${COMMON_SOURCES}
  ${LIB_SOURCE}
  )
target_link_libraries(MmalEncoder
  ${USER_LAND}/build/lib/libmmal.so
  ${USER_LAND}/build/lib/libmmal_core.so
  ${USER_LAND}/build/lib/libmmal_util.so
  ${USER_LAND}/build/lib/libvcos.so
  ${USER_LAND}/build/lib/libbcm_host.so
  pthread
  )

# make test main exe object
add_executable(MmalMain ${COMMON_SOURCES} ${BIN_SOURCE})
target_link_libraries(MmalMain
  ${USER_LAND}/build/lib/libmmal.so
  ${USER_LAND}/build/lib/libmmal_core.so
  ${USER_LAND}/build/lib/libmmal_util.so
  ${USER_LAND}/build/lib/libvcos.so
  ${USER_LAND}/build/lib/libbcm_host.so
  pthread
  MmalEncoder
  )

