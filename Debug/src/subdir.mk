################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/SPC_main.cpp \
../src/stereo_match.cpp 

OBJS += \
./src/SPC_main.o \
./src/stereo_match.o 

CPP_DEPS += \
./src/SPC_main.d \
./src/stereo_match.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include -I"/home/loyd-hook/0Projects/SV/software/eclipse_ws/StereoPointCloud/hdr" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


