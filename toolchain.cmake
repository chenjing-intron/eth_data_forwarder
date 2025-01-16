# 设置目标系统名称
set(CMAKE_SYSTEM_NAME Linux)

# 设置目标处理器架构
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 指定交叉编译器的路径
set(CMAKE_C_COMPILER /chenjing/gcc/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /chenjing/gcc/bin/aarch64-linux-gnu-g++)

# 指定交叉编译器的根目录（可选）
#set(CMAKE_FIND_ROOT_PATH /path/to/your/sysroot)

# 设置编译器标志（可选）
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon")

# 设置查找库和头文件的规则
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
