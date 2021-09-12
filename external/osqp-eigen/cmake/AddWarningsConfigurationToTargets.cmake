

include(CMakeParseArguments)

function(ADD_WARNINGS_CONFIGURATION_TO_TARGETS)

  set(_options PRIVATE
               PUBLIC
               INTERFACE)
  set(_oneValueArgs )
  set(_multiValueArgs TARGETS)

  cmake_parse_arguments(_AWC2T "${_options}"
                               "${_oneValueArgs}"
                               "${_multiValueArgs}"
                               "${ARGN}")

  # Check the kind of visibility requested
  set(VISIBILITIES)
  if (${_AWC2T_PRIVATE})
    list(APPEND VISIBILITIES PRIVATE)
  endif()
  if (${_AWC2T_PUBLIC})
    list(APPEND VISIBILITIES PUBLIC)
  endif()
  if (${_AWC2T_INTERFACE})
    list(APPEND VISIBILITIES INTERFACE)
  endif()


  foreach(TARGET ${_AWC2T_TARGETS})
    foreach(VISIBILITY ${VISIBILITIES})
      #setting debug options
      set(COMPILE_OPTIONS "")
      if(MSVC)
        ###
        list(APPEND COMPILE_OPTIONS "/W3")
      else()
        ##Other systems
        if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
          list(APPEND COMPILE_OPTIONS "-Weverything")
          list(APPEND COMPILE_OPTIONS "-pedantic")
          list(APPEND COMPILE_OPTIONS "-Wnon-virtual-dtor")
          list(APPEND COMPILE_OPTIONS "-Woverloaded-virtual")
          list(APPEND COMPILE_OPTIONS "-Wc++11-extensions")
          #Remove some other warnings on paddings
          list(APPEND COMPILE_OPTIONS "-Wno-padded")
          list(APPEND COMPILE_OPTIONS "-Wno-cast-align")
          list(APPEND COMPILE_OPTIONS "-Wno-c++98-compat")

        elseif(${CMAKE_COMPILER_IS_GNUCC})
          list(APPEND COMPILE_OPTIONS "-Wall")
          list(APPEND COMPILE_OPTIONS "-pedantic")
          list(APPEND COMPILE_OPTIONS "-Wextra")
          list(APPEND COMPILE_OPTIONS "-Woverloaded-virtual")
          list(APPEND COMPILE_OPTIONS "-Wconversion")
        endif()
      endif()

      target_compile_options(${TARGET} ${VISIBILITY} "$<$<CONFIG:Debug>:${COMPILE_OPTIONS}>")

    endforeach()
  endforeach()

endfunction()

