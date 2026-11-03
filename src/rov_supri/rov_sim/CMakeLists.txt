cmake_minimum_required(VERSION 3.8)
project(rov_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

get_filename_component(ARDUSUPRI_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/Ardusupri" REALPATH)

option(BUILD_ARDUSUB_SITL "Build ArduSub SITL with waf" ON)
if(BUILD_ARDUSUB_SITL)
  execute_process(
    COMMAND python3 -c "import em"
    RESULT_VARIABLE EM_RESULT
    OUTPUT_QUIET ERROR_QUIET)
  if(NOT EM_RESULT EQUAL 0)
    message(FATAL_ERROR "Missing Python module 'empy'. Run ./setup.sh first.")
  endif()

  if(NOT EXISTS "${ARDUSUPRI_DIR}/build/sitl/c4che/_cache.py")
    execute_process(
      COMMAND python3 ./waf configure --board sitl
      WORKING_DIRECTORY "${ARDUSUPRI_DIR}"
      RESULT_VARIABLE WAF_CONFIGURE_RESULT)
    if(NOT WAF_CONFIGURE_RESULT EQUAL 0)
      message(FATAL_ERROR "waf configure failed in ${ARDUSUPRI_DIR}")
    endif()
  endif()

  add_custom_target(build_ardusub ALL
    COMMAND python3 ./waf sub
    WORKING_DIRECTORY "${ARDUSUPRI_DIR}"
    USES_TERMINAL)
endif()

configure_file(
  launch/sitl.launch.xml.in
  ${CMAKE_CURRENT_BINARY_DIR}/launch/sitl.launch.xml
  @ONLY)

install(
  DIRECTORY models worlds launch config params
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
  PATTERN "*.in" EXCLUDE)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/launch/sitl.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
