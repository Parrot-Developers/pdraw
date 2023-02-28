add_library(pdraw-vsink STATIC src/pdraw_vsink.c)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set_target_properties(pdraw-vsink PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(pdraw-vsink PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(pdraw-vsink PRIVATE "PDRAW_VSINK_API_EXPORTS")
target_compile_options(pdraw-vsink PRIVATE "-fvisibility=hidden")
target_compile_options(pdraw-vsink PRIVATE "-std=gnu99")

target_link_libraries(pdraw-vsink
                        futils
                        media-buffers
                        media-buffers-memory
                        media-buffers-memory-generic
                        pdraw
                        pomp
                        ulog
)
