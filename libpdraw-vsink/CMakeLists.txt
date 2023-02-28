cmake_minimum_required(VERSION 3.15)
project(pdraw-vsink VERSION 1.0)

if (APPLE)
  set(rbase "@loader_path")
else ()
  set(rbase "$ORIGIN")
endif ()



set(CMAKE_SKIP_BUILD_RPATH FALSE)


# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

file(RELATIVE_PATH lib_dir
    "${CMAKE_INSTALL_PREFIX}/bin"
    "${CMAKE_INSTALL_PREFIX}/lib")

set(CMAKE_INSTALL_RPATH "${rbase};${rbase}/${lib_dir}"
    CACHE STRING "Install RPATH")

add_library(${PROJECT_NAME} src/pdraw_vsink.c)

# add_executable(${PROJECT_NAME}-test src/pdraw_vsink.c)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "PDRAW_VSINK_API_EXPORTS")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=gnu99")

target_link_libraries(${PROJECT_NAME}
                      futils
                      media-buffers
                      media-buffers-memory
                      media-buffers-memory-generic
                      pdraw
                      pomp
                      ulog
)

set(${PROJECT_NAME}-headers
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# to get all header files correctly
install(
    DIRECTORY ${${PROJECT_NAME}-headers} DESTINATION include/${PROJECT_NAME}
)

