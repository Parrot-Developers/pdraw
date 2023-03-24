cmake_minimum_required(VERSION 3.15)
project(pdraw-vsink VERSION 1.0)

add_library(${PROJECT_NAME} SHARED src/pdraw_vsink.c)

# checks if set up rpath exists for install
if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set_target_properties(${PROJECT_NAME} PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "PDRAW_VSINK_API_EXPORTS")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=gnu99")

target_link_libraries(${PROJECT_NAME}

                     PRIVATE
                      futils
                      media-buffers-memory
                      pomp
                      ulog
                      pdraw
                      media-buffers
                      media-buffers-memory-generic
                      # PUBLIC
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

