macro(ConfigExternalProject NAME)
    set(EP_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/usr" CACHE PATH "")

    file(MAKE_DIRECTORY "${EP_PREFIX}")

    set(params)
    foreach(arg ${ARGN})
        string(REPLACE " " "\\ " arg "${arg}")
        #message("ARG: ${arg}")
        set(params "${params} ${arg}")
    endforeach()

    if(MSVC)
        set(BUILD_TYPES Debug Release)
    else()
        set(BUILD_TYPES ${CMAKE_BUILD_TYPE})
    endif()

    if(BUILD_TYPES STREQUAL "")
        set(BUILD_TYPES Release)
    endif()

    foreach(BUILD_TYPE ${BUILD_TYPES})

        set(bindir "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/${BUILD_TYPE}")
        file(MAKE_DIRECTORY "${bindir}")
        set(tmpfile "${bindir}/CMakeLists.txt")

        if(MSVC)
            # we pass the default C(XX) flags and the ones (possibly) defined by the user
            set(parallel_cl_cxx_build -DCMAKE_CXX_FLAGS:STRING=\${CMAKE_CXX_FLAGS}\\\ \\\${CMAKE_CXX_FLAGS}\\\ /MP)
            set(parallel_cl_c_build -DCMAKE_C_FLAGS:STRING=\${CMAKE_C_FLAGS}\\\ \\\${CMAKE_C_FLAGS}\\\ /MP)
        else()
            # we pass the default C(XX) flags and the ones (possibly) defined by the user
            set(parallel_cl_cxx_build -DCMAKE_CXX_FLAGS:STRING=\${CMAKE_CXX_FLAGS}\\\ \\\${CMAKE_CXX_FLAGS})
            set(parallel_cl_c_build -DCMAKE_C_FLAGS:STRING=\${CMAKE_C_FLAGS}\\\ \\\${CMAKE_C_FLAGS})
        endif()


        file(WRITE "${tmpfile}" 
            "cmake_minimum_required(VERSION 3.0)\n"
            "include(ExternalProject)\n"
            "ExternalProject_Add(${NAME} ${params}
                                 INSTALL_DIR \"${EP_PREFIX}\"
                                 CMAKE_ARGS -DCMAKE_PREFIX_PATH:PATH=${EP_PREFIX} -DCMAKE_INSTALL_PREFIX:PATH=${EP_PREFIX} -Wno-dev -DCMAKE_MODULE_PATH:PATH=${CMAKE_CURRENT_SOURCE_DIR}/cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_DEBUG_POSTFIX:STRING=d
                                 CMAKE_CACHE_ARGS ${parallel_cl_cxx_build} ${parallel_cl_c_build} -DEP_PREFIX:PATH=${EP_PREFIX}
                                 PREFIX \"${bindir}\")\n")

        message(STATUS "Configuring ${NAME} - ${BUILD_TYPE}")

        execute_process(COMMAND ${CMAKE_COMMAND} "${bindir}"
                        WORKING_DIRECTORY "${bindir}"
                        RESULT_VARIABLE result)
                        #                    OUTPUT_FILE ${bindir}/config_output.txt
                        #ERROR_FILE ${bindir}/config_error.txt
                        #                    OUTPUT_VARIABLE errmsg
                        #ERROR_VARIABLE errmsg)
        if(NOT "${result}" STREQUAL "0")
            message(FATAL_ERROR "Error configuring ${NAME}: ${result} ${errmsg}")
        endif()

        message(STATUS "Building ${NAME} - ${BUILD_TYPE}")

        if(MSVC)
            set(PARALLEL_BUILD /maxcpucount)
        else()
            include(ProcessorCount)
            ProcessorCount(N)
            if(NOT N EQUAL 0)
                set(PARALLEL_BUILD -j${N})
            endif()
        endif()

        execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${BUILD_TYPE} -- ${PARALLEL_BUILD}
                        WORKING_DIRECTORY "${bindir}"
                        RESULT_VARIABLE result
                        OUTPUT_FILE ${bindir}/build_output.txt
                        ERROR_FILE ${bindir}/build_error.txt)
                        #OUTPUT_VARIABLE errmsg
                        #ERROR_VARIABLE errmsg)

        if(NOT "${result}" STREQUAL "0")
            message(FATAL_ERROR "Error building ${NAME}: ${result} ${errmsg}")
        endif()
    endforeach()

    find_package(${NAME} QUIET NO_DEFAULT_PATH PATHS "${EP_PREFIX}")
    if(${NAME}_FOUND)
        message(STATUS "Using built-in ${NAME} library")
    endif()

    include_directories(${EP_PREFIX}/include)
endmacro()
