# -
一个通过stm32f103c8t6作为主控，使用SPL06-001气压计定高，MPU6050姿态控制，HC-06为串口通信，实现查看数据波形调参，2.4gNRF24l01+实现遥控控制姿态飞行，采用8520空心杯电机+65mm桨叶，mos管驱动
这是通过波形以及对比实验不同桨叶震动带来对加速度计原始值影响和滤波后的效果，还有角度，固定测试，气压计上升电机启动，
<img width="2160" height="2880" alt="IMG_20261020_082906" src="https://github.com/user-attachments/assets/20217428-f554-428a-8aca-15c8ee86890e" />
<img width="2160" height="2880" alt="IMG_20261020_084806" src="https://github.com/user-attachments/assets/5928ce5f-8f32-4b0e-8fe9-7b77e0a1e9b5" />
<img width="2160" height="2880" alt="IMG_20261020_083050" src="https://github.com/user-attachments/assets/753b7eef-83ea-4ee7-a99d-1229f0e02de4" />
<img width="2160" height="2880" alt="IMG_20261020_084806" src="https://github.com/user-attachments/assets/33752c08-da23-440c-8d2f-4d5053a0f198" />
实测下来是65mm桨叶兼顾推力和震动影响效果平衡
当然MPU6050带来的问题也坑，这是假芯片+打磨翻新后的效果最差
<img width="3072" height="4096" alt="IMG_20261016_212321" src="https://github.com/user-attachments/assets/81067a65-2671-4bea-8c06-1150d34ff6f1" />
<img width="1440" height="3200" alt="Screenshot_2026-10-16-21-47-13-841_com tencent mm" src="https://github.com/user-attachments/assets/dd89658f-8652-47c0-a462-bd5cba0a1b01" />
这是好的MPU6050，对震动影响可以对比，通过清晰丝印可以判断是否完好，是否翻新
<img width="4096" height="3072" alt="IMG_20261016_211938" src="https://github.com/user-attachments/assets/73b499a8-65c6-4427-8ad0-d61c675c019f" />
<img width="1440" height="3200" alt="Screenshot_2026-10-16-21-43-00-376_com tencent mm" src="https://github.com/user-attachments/assets/174d9f61-5a41-4a86-8d64-00722368150c" />
<img width="1440" height="3200" alt="Screenshot_2026-10-16-21-44-23-589_com tencent mm" src="https://github.com/user-attachments/assets/59367605-8849-4372-8396-0d131e0db7cc" />
组装
用2个150mm轻木条组装机身，用热熔胶固定机身
<img width="1844" height="4096" alt="IMG_20261021_175309" src="https://github.com/user-attachments/assets/5a7f0c26-4288-4fed-b79f-e8e378c8f82c" />
<img width="4096" height="3072" alt="IMG_20261017_002435" src="https://github.com/user-attachments/assets/8b1baa6c-25c6-4072-9ba2-3654d897f168" />
<img width="4096" height="3072" alt="IMG_20260907_232924" src="https://github.com/user-attachments/assets/64603421-2210-4618-bcca-5c7366e13772" />

采用模块和mcu集成到板子上面，初版因为Pmos开关电源没有把通过mos管的电机电流分出去，不经过mos管，导致开关电源失效，新版把电机电流不通过mos管，这样mcu断电，mos管也不会开启导致电机旋转。
立创原理图
<img width="416" height="316" alt="屏幕截图 2026-10-22 205039" src="https://github.com/user-attachments/assets/be556560-c8f7-47c6-ae7f-1e62d204c2de" />
<img width="528" height="226" alt="屏幕截图 2026-10-22 205143" src="https://github.com/user-attachments/assets/75ed8abd-d9ed-42f5-a4e0-d12185ffdee5" />
<img width="821" height="426" alt="屏幕截图 2026-10-22 205138" src="https://github.com/user-attachments/assets/a50de157-cf30-4ebd-aefc-f5ea0addaad3" />
下面是对应遥控PCB和实物图，也是采用摇杆电位器PS2或者Ps5ADC读取电压来映射到油门和姿态来控制
<img width="4096" height="3072" alt="IMG_20261022_205317" src="https://github.com/user-attachments/assets/2e33f681-f2fd-4528-8838-d74eb6f58b1d" />
<img width="611" height="482" alt="屏幕截图 2026-10-22 205712" src="https://github.com/user-attachments/assets/d43f5c4f-b352-4517-9b29-f357e47cd154" />
<img width="528" height="226" alt="屏幕截图 2026-10-22 205143" src="https://github.com/user-attachments/assets/fe68dc2b-5269-4c9e-8a5d-5ce7c2af14d3" />
<img width="821" height="426" alt="屏幕截图 2026-10-22 205138" src="https://github.com/user-attachments/assets/ccb2e93d-733b-4745-93cb-7e67962b729f" />


这是实际飞行效果

https://github.com/user-attachments/assets/13fcd768-962f-4ac4-8a0a-9ec127c3904c

