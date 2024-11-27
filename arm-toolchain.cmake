# 设置编译器
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm)

# 指定交叉编译工具链的前缀
SET(CMAKE_C_COMPILER /home/lhy/codecplus/arm-ca53-linux-gnueabihf-8.4/bin/arm-ca53-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /home/lhy/codecplus/arm-ca53-linux-gnueabihf-8.4/bin/arm-ca53-linux-gnueabihf-g++)
SET(CMAKE_STRIP /home/lhy/codecplus/arm-ca53-linux-gnueabihf-8.4/bin/arm-ca53-linux-gnueabihf-strip)

# 设置 sysroot，如果有
SET(CMAKE_SYSROOT /home/lhy/codecplus/arm-ca53-linux-gnueabihf-8.4/arm-ca53-linux-gnueabihf/sysroot)

# 设置 C 和 C++ 编译器标志（如果有特定需求）
SET(CMAKE_C_FLAGS "-march=armv8-a")
SET(CMAKE_CXX_FLAGS "-march=armv8-a")

