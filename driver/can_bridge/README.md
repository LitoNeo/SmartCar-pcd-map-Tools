# USB2CAN_alystii driver
This module is used to drive the USB2CAN_alystii device

## 1.USB2CAN_alystii介绍

## 2.USB2CAN_alystii简单使用
将自动从系统库/自定义库中查找libcontrolcan.so, 请提前将库文件放入搜索路径中,或在CMakeList.txt中指定查找路径
## 3. 固定设备权限
在Linux系统下,USB2CAN_alystii需要设备普通用户可读写权限以使用该设备,而每次插入设备都会被分配不同的设备端口号,无法通过脚本来执行,因此需要使用`udev`的规则,通过设备的`idVendor`和`idProduct`自动识别该设备,并设置可读写权限.
执行:
```
echo  'ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", MODE:="0666", GROUP:="dialout",  SYMLINK+="usb2can_alystii"' >/etc/udev/70-usb2can-alystii.rules
 ```
***注:有的教程里会加入KERNEL="ttyUSB"等之类,但是KERNEL值设置错误会导致无法自动识别设备,因此在不确定Kernel Value的情况下,可以不加KERNEL字段,同样能够达到预期目的 ***
然后重新拔插设备即可.   

**TODO::研究udev规则机制**

## 4. USB2CAN_alystii驱动主要接口函数说明

## 5. Subscriber/Publisher & Topic
**Subscriber:**  

| type | topic |
| :--- | :---  |
| can_msgs::ecu | "/ecu" |  

**Publisher:**  

| type | topic |
| :--- | :---  |
| can_msgs::vehicle_status | "/vehicle_status" |
| can_msgs::battery | "/battery" |



