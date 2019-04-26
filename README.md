# Robot Kinematics Calculator for Philos Robot
This program, *robot_kinematics*, requires the [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) library  in order to compile. Note that the downloaded library might have a different folder name from what was written in the code (Eigen), therefore make sure the two names are the same, otherwise the compiler will throw you an error.
A sample command for compiling the source code is `g++ -std=c++11 ./src/arm.cpp ./src/kinematics_calculator.cpp -I /usr/include/ -I ./include/ -o robot_kinematics` assuming the [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) library is installed under */usr/include*.

This program is to be used in conjunction with [philos_project](https://github.com/YanzhouWang/philos_project) to reflect the robot design.
