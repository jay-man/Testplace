# Install script for directory: /home/jannik/eigen/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/jannik/eigen/Eigen/Dense"
    "/home/jannik/eigen/Eigen/Geometry"
    "/home/jannik/eigen/Eigen/Jacobi"
    "/home/jannik/eigen/Eigen/QR"
    "/home/jannik/eigen/Eigen/Sparse"
    "/home/jannik/eigen/Eigen/KLUSupport"
    "/home/jannik/eigen/Eigen/UmfPackSupport"
    "/home/jannik/eigen/Eigen/Cholesky"
    "/home/jannik/eigen/Eigen/SparseCore"
    "/home/jannik/eigen/Eigen/SparseLU"
    "/home/jannik/eigen/Eigen/IterativeLinearSolvers"
    "/home/jannik/eigen/Eigen/Core"
    "/home/jannik/eigen/Eigen/QtAlignedMalloc"
    "/home/jannik/eigen/Eigen/CholmodSupport"
    "/home/jannik/eigen/Eigen/SparseQR"
    "/home/jannik/eigen/Eigen/PardisoSupport"
    "/home/jannik/eigen/Eigen/LU"
    "/home/jannik/eigen/Eigen/OrderingMethods"
    "/home/jannik/eigen/Eigen/PaStiXSupport"
    "/home/jannik/eigen/Eigen/Householder"
    "/home/jannik/eigen/Eigen/SuperLUSupport"
    "/home/jannik/eigen/Eigen/Eigenvalues"
    "/home/jannik/eigen/Eigen/StdList"
    "/home/jannik/eigen/Eigen/SVD"
    "/home/jannik/eigen/Eigen/Eigen"
    "/home/jannik/eigen/Eigen/StdVector"
    "/home/jannik/eigen/Eigen/SPQRSupport"
    "/home/jannik/eigen/Eigen/StdDeque"
    "/home/jannik/eigen/Eigen/MetisSupport"
    "/home/jannik/eigen/Eigen/SparseCholesky"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/jannik/eigen/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

