# OpenCV libraries without relying on OpenCV_LIBS: some builds (OpenCV 4.13 from source
# in /usr/local on the Jetson) set no variables. Use OpenCV's imported targets, else the
# library inside the same install (never a same-named library of another OpenCV).
function(fusion_opencv_libs out)
  get_filename_component(_prefix "${OpenCV_DIR}/../../.." ABSOLUTE)
  set(_libs "")
  foreach(_m ${ARGN})
    if(TARGET ${_m})
      list(APPEND _libs ${_m})
    else()
      find_library(_lib_${_m} NAMES ${_m} HINTS ${_prefix}/lib ${_prefix}/lib64 NO_DEFAULT_PATH)
      if(NOT _lib_${_m})
        message(FATAL_ERROR "${_m} not found in ${_prefix}/lib (OpenCV from ${OpenCV_DIR})")
      endif()
      list(APPEND _libs ${_lib_${_m}})
    endif()
  endforeach()
  set(${out} ${_libs} PARENT_SCOPE)
endfunction()
