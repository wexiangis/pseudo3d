# 编译器选择
CC = gcc

# ----- 文件夹列表 -----

DIR_SRC = src
DIR_COMMON = common
DIR_SENSOR = sensor
DIR_SENSOR_MPU = sensor/mpu6050
DIR_UI = ui

# obj目录用于缓存编译生成的.o文件
DIR_OBJ = obj

# ----- 文件夹编译及.o文件转储 -----

# 头文件目录列表
INC = -I$(DIR_SRC) -I$(DIR_COMMON) -I$(DIR_SENSOR) -I$(DIR_SENSOR_MPU) -I$(DIR_UI)

%.o:../$(DIR_SRC)/%.c
	@$(CC) -Wall -c $< $(INC) -o $@
%.o:../$(DIR_COMMON)/%.c
	@$(CC) -Wall -c $< $(INC) -o $@
%.o:../$(DIR_SENSOR)/%.c
	@$(CC) -Wall -c $< $(INC) -o $@
%.o:../$(DIR_SENSOR_MPU)/%.c
	@$(CC) -Wall -c $< $(INC) -o $@
%.o:../$(DIR_UI)/%.c
	@$(CC) -Wall -c $< $(INC) -o $@

# ----- obj中的.o文件统计 -----

obj += ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_SRC)/*.c}}}
obj += ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_COMMON)/*.c}}}
obj += ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_SENSOR)/*.c}}}
obj += ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_SENSOR_MPU)/*.c}}}
obj += ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_UI)/*.c}}}

#----- 把所有.o文件链接,最终编译 -----

out: $(obj)
	@$(CC) -Wall -o out $(obj) $(INC) -lm -lpthread

clean:
	@rm ./obj/* out