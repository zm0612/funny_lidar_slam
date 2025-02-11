IF(UNIX)
  MESSAGE(STATUS "Searching for g2o ...")
  FIND_PATH(G2O_INCLUDE_DIR
    NAMES core math_groups types
    PATHS /usr/local /usr
    PATH_SUFFIXES include/g2o include)
  IF (G2O_INCLUDE_DIR)
    MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  ENDIF ()

  FIND_LIBRARY(G2O_CORE_LIB
    NAMES g2o_core g2o_core_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_STUFF_LIB
    NAMES g2o_stuff g2o_stuff_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_TYPES_SLAM2D_LIB
    NAMES g2o_types_slam2d g2o_types_slam2d_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_TYPES_SLAM3D_LIB
    NAMES g2o_types_slam3d g2o_types_slam3d_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_CHOLMOD_LIB
    NAMES g2o_solver_cholmod g2o_solver_cholmod_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_PCG_LIB
    NAMES g2o_solver_pcg g2o_solver_pcg_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_CSPARSE_LIB
    NAMES g2o_solver_csparse g2o_solver_csparse_rd
    PATHS /usr/local /usr
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_INCREMENTAL_LIB
    NAMES g2o_incremental g2o_incremental_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_CSPARSE_EXTENSION_LIB
    NAMES g2o_csparse_extension g2o_csparse_extension_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib)

  SET(G2O_LIBRARIES ${G2O_CSPARSE_EXTENSION_LIB}
                    ${G2O_CORE_LIB}
                    ${G2O_STUFF_LIB}
                    ${G2O_TYPES_SLAM2D_LIB}
                    ${G2O_TYPES_SLAM3D_LIB}
                    ${G2O_SOLVER_CHOLMOD_LIB}
                    ${G2O_SOLVER_PCG_LIB}
                    ${G2O_SOLVER_CSPARSE_LIB}
                    ${G2O_INCREMENTAL_LIB})

  IF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    SET(G2O_FOUND "YES")
    MESSAGE(STATUS "Found libg2o: ${G2O_LIBRARIES}")
  ELSE()
    IF(NOT G2O_LIBRARIES)
      message(FATAL_ERROR "Could not find libg2o!")
    ENDIF()
    IF(NOT G2O_INCLUDE_DIR)
      message(FATAL_ERROR "Could not find g2o include directory!")
    ENDIF()
  ENDIF()
ENDIF(UNIX)
