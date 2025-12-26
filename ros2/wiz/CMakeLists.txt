cmake_minimum_required(VERSION 3.14)
project(wiz VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

# Find cargo
find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found. Please install Rust: https://rustup.rs/")
endif()

# wiz project root directory (relative to this CMakeLists.txt)
get_filename_component(WIZ_PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# Rust binary paths
set(WIZ_SERVER_BIN "${WIZ_PROJECT_DIR}/target/release/wiz-server")
set(WIZ_FRONTEND_BIN "${WIZ_PROJECT_DIR}/target/release/wiz-frontend")

# Build Rust binaries using cargo
add_custom_command(
  OUTPUT ${WIZ_SERVER_BIN} ${WIZ_FRONTEND_BIN}
  COMMAND ${CARGO} build --release -p wiz-server -p wiz-frontend
  WORKING_DIRECTORY ${WIZ_PROJECT_DIR}
  COMMENT "Building wiz Rust binaries with cargo..."
  VERBATIM
)

add_custom_target(wiz_rust_binaries ALL
  DEPENDS ${WIZ_SERVER_BIN} ${WIZ_FRONTEND_BIN}
)

# Install binaries
install(PROGRAMS
  ${WIZ_SERVER_BIN}
  ${WIZ_FRONTEND_BIN}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
