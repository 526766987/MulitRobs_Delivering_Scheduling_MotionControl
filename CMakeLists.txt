# CMakeList.txt: HW2023Project 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

project ("HW2023Project")

# 将源代码添加到此项目的可执行文件。
add_executable (HW2023Project "Main.cpp" "Main.h" "Path.h" "Path.cpp"  "Schedule.cpp" "Schedule.h" "Global.h" "Global.cpp" "Calculate.h" "Calculate.cpp" "Out.h" "Out.cpp" "Attack.h" "Attack.cpp" "Map.h" "Map.cpp" "Motion.h" "Motion.cpp" "Defend.h" "Defend.cpp" "Readin.h" "Readin.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET HW2023Project PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
