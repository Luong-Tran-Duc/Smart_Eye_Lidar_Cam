execute_process(COMMAND "/home/aa-04/Smart_Eye_Lidar_Cam/build/vikit_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/aa-04/Smart_Eye_Lidar_Cam/build/vikit_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
