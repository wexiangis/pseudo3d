
## 基于mpu6050模块得到的姿态信息, 姿态信息输入到一个伪3D引擎, 并显示于 /dev/fb0 设备中

---

* 1. 看不到输出效果

* 尝试组合键"ctrl+alt+f1" 或者 命令行输入"sudo init 3 &" 切换命令行显示模式

---

* 2. 怎么恢复显示界面

* 尝试组合键"ctrl+alt+f7" 或者 命令行输入"sudo init 5 &" 切换界面显示模式

---

* 3. 没有传感器,只想在ubuntu上看下伪3D显示效果

* 在 src/main.c 中注释掉行 "#define ENABLE_MPU6050 1", 运行时通过按键1,2,3,q,w,e,a,s,d,z,x,c来控制图形旋转和翻转,r键复位.

---

* 4. 切换兼容mpu9250

* 在 sensor/mpu6050/inv_mpu.h 最上面, 通过切换宏定义 #define MPU6050 或者 #define MPU9250 来切换模块.

---

* 5. 切换dmp解算和自己的四元数解算

* 在 src/posture.c 最上面, 通过定义 #define PE_QUATERNION 来启用本地四元数解算, 但需自行调整 quaternion() 函数内的 Kp 和 Ki 参数.
