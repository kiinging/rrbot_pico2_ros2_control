add_library(pca9685
	PCA9685_servo_driver.cpp
	PCA9685_servo.cpp
)

target_link_libraries(pca9685 PUBLIC
		pico_stdlib
		hardware_i2c
		)


target_include_directories(pca9685 PUBLIC
   // ${CMAKE_CURRENT_SOURCE_DIR}/../../include/pca9685
	${CMAKE_SOURCE_DIR}/include/pca9685
)