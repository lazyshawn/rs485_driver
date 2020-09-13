
cmake_minimum_required(VERSION 3.0)

project(t_motor)


#add_executable (RMD_485_Driver RMD_485_Driver.cpp)
#target_link_libraries (RMD_485_Driver ${PCL_LIBRARIES}  ${CGAL_LIBS})


# add_executable(RMD_485_Driver   
#       usb_to_485.h
#       RMD_485_Driver.cpp
#   )


#target_link_libraries(RMD_485_Driver)


add_executable(usb_to_485   
      usb_to_485.cpp
      RMD_485_Driver.h 
      RMD_485_Driver.cpp
  )


target_link_libraries(usb_to_485)
