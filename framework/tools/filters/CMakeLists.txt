cmake_minimum_required(VERSION 2.6.0)

if (NOT SLAMBENCH_INCLUDE_DIR)
  message( FATAL_ERROR "SLAMBENCH_INCLUDE_DIR not found")
endif()

add_library(identity-filter SHARED identity.cpp )
target_link_libraries(identity-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(identity-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 

add_library(skip-filter    SHARED  skip_frame.cpp )
target_link_libraries(skip-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(skip-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 
        
add_library(intensity-filter    SHARED  intensity_kldiv.cpp )
target_link_libraries(intensity-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(intensity-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 
        
add_library(rgb-filter    SHARED  rgb_kldiv.cpp )
target_link_libraries(rgb-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(rgb-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 
        
add_library(naive-filter    SHARED  naive.cpp )
target_link_libraries(naive-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(naive-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 

add_library(depth-filter    SHARED  depth_kldiv.cpp )
target_link_libraries(depth-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(depth-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 

add_library(combined-filter    SHARED  combined_di_kldiv.cpp )
target_link_libraries(combined-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(combined-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 
