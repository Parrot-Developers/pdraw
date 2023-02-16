add_library(pdraw-vsink src/pdraw_vsink.c)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

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
