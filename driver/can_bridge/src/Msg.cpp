#include "Msg.h"

namespace USB2CAN
{
// -*-*-*-*- SendMsg -*-*-*-*-

void SendMsg::initCanMsg()
{
    bzero(canMsg.Data, BUFFER_SIZE);
    canMsg.ID = 0x120;
    canMsg.SendType = 0x01;   // 0:正常发送,发送失败则重发  1:只发送一次
    canMsg.RemoteFlag = 0x00; // 0:数据帧  1:远程帧(数据段空)
    canMsg.ExternFlag = 0x00; // 0:标准帧  1:扩展帧
    canMsg.DataLen = 0x08;

    this->setGradient(0);          // 设置路面坡度
    this->setAccMode(1);           // 设置加速档位
    this->setDriveMode(AUTO_MODE); // 设置驾驶模式 ： 自动驾驶
    this->setBrakeMode(1);         // 设置减速档位
}

void SendMsg::setDriveMode(DriveMode driveMode)
{
    writeInt(canMsg.Data, SCU_DRIVE_MODE_REQ_OFFSET, SCU_DRIVE_MODE_REQ_LENGTH, driveMode);
}

void SendMsg::setAccMode(int v)
{
    writeInt(canMsg.Data, SCU_ACC_MODE_OFFSET, SCU_ACC_MODE_LENGTH, v);
}

void SendMsg::setBrakeMode(int v)
{
    writeInt(canMsg.Data, SCU_BRAKE_MODE_OFFSET, SCU_BRAKE_MODE_LENGTH, v);
}

void SendMsg::setSpeed(double v)
{
    int speed = (int)lround(v * 10);
    writeInt(canMsg.Data, SCU_TARGET_SPEED_OFFSET, SCU_TARGET_SPEED_LENGTH, speed);
}

void SendMsg::setWheelAngle(double angle)
{
    short v = angle * 10; // 分辨率0.1,转换成整数
    writeInt(canMsg.Data, SCU_STEERING_WHEEL_ANGLE_OFFSET, SCU_STEERING_WHEEL_ANGLE_LENGTH, v);
}

void SendMsg::setShiftLevel(ShiftLevel shift_level)
{
    writeInt(canMsg.Data, SCU_SHIFT_LEVEL_OFFSET, SCU_SHIFT_LEVEL_LENGTH, shift_level);
}

void SendMsg::setGradient(double v)
{
    int iv = v * 2; // 因为can里面gradient的单位是0.5度
    writeInt(canMsg.Data, SCU_GRADIENT_OFFSET, SCU_GRADIENT_LENGTH, iv);
}

void SendMsg::setEBrake(bool need)
{
    int v = 0;
    if (need)
    {
        v = 1;
    }
    writeInt(canMsg.Data, SCU_EBRAKE_OFFSET, SCU_EBRAKE_LENGTH, v);
}

void SendMsg::print()
{
    fprintf(stdout, "send ecu/CCUMsg: "); // printf:格式化输出到屏幕stdout; fprintf格式化输出到文件FILE; sprintf格式化输出到字符创char*
    fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
    fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
    this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
    this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
    fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
    fprintf(stdout, "Data: ");
    for (int i = 0; i < 8; i++)
    {
        fprintf(stdout, "%02X ", this->canMsg.Data[i]);
    }
    fprintf(stdout, "...\n");
}

void SendMsg::rosMsg2canMsg()
{
    this->setBrakeMode(ecuMsg.brake);

    this->setShiftLevel(ShiftLevel(ecuMsg.shift));

    this->setSpeed(ecuMsg.motor);

    this->setWheelAngle((ecuMsg.steer + this->pre_steer) / 2);

    this->setEBrake(ecuMsg.brake);
}

VCI_CAN_OBJ SendMsg::getMessage()
{
    this->rosMsg2canMsg();
    VCI_CAN_OBJ return_msg = this->canMsg;
    return return_msg;
}

// -*-*-*-*- Received canMsg: vehicle_status -*-*-*-*-

int VehicleStatusMsg::shiftLevel()
{
    return readAsInt(this->canMsg.Data, CCU_SHIFT_LEVEL_OFFSET, CCU_SHIFT_LEVEL_LENGTH);
}

double VehicleStatusMsg::speed()
{
    double speed = readAsInt(this->canMsg.Data, CCU_SPEED_OFFSET, CCU_SPEED_LENGTH) / 10.0; // 单位 m/s
    return speed;
}

SteerDirection VehicleStatusMsg::wheelDirection()
{
    int dir = readBit(this->canMsg.Data, CCU_WHEEL_DIRECTION_OFFSET);
    if (dir == 0)
        return left;
    return right;
}

double VehicleStatusMsg::wheelAngle()
{
    int wheel_angle_right = readBit(this->canMsg.Data, CCU_WHEEL_DIRECTION_OFFSET);
    double angle = readAsInt(this->canMsg.Data, CCU_WHEEL_ANGLE_OFFSET, CCU_WHEEL_ANGLE_LEGNTH) / 31.48; // 大车±85对应实际±27°,小车未知(待确认)
    if (wheel_angle_right == 1)
    {
        return angle;
    }
    else
    {
        return -angle;
    }
}

int VehicleStatusMsg::driveMode()
{
    return readAsInt(this->canMsg.Data, CCU_DRIVE_MODE_OFFSET, CCU_DRIVE_MODE_LENGTH);
}

int VehicleStatusMsg::accLevel()
{
    return readAsInt(this->canMsg.Data, CCU_ACCELERATE_LEVEL_OFFSET, CCU_ACCELERATE_LEVEL_LENGTH);
}

int VehicleStatusMsg::brakeLevel()
{
    return readAsInt(this->canMsg.Data, CCU_BRAKE_LEVEL_OFFSET, CCU_BRAKE_LEVEL_LENGTH);
}

double VehicleStatusMsg::totalOdometer()
{
    double total_odometer = readAsInt(this->canMsg.Data, CCU_TOTAL_ODOMETER_OFFSET, CCU_TOTAL_ODOMETER_LENGTH);
    return total_odometer;
}

std::string VehicleStatusMsg::toString()
{
    std::string msg;
    std::stringstream ss;
    ss << "RevMsg speed: " << this->speed() << ", shiftLevel: " << this->shiftLevel() << ", wheelAngle: " << this->wheelAngle()
       << ", driveMode: " << this->driveMode() << ", accLevel: " << this->accLevel() << ", brakeLevel: " << this->brakeLevel()
       << ", totalOdometer: " << this->totalOdometer();
    ss >> msg;
    ss.str();
    return msg;
}

void VehicleStatusMsg::print()
{
    fprintf(stdout, "Recv Feedback: "); // printf:格式化输出到屏幕stdout; fprintf格式化输出到文件FILE; sprintf格式化输出到字符创char*
    fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
    fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
    this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
    this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
    fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
    fprintf(stdout, "Data: ");
    for (int i = 0; i < 8; i++)
    {
        fprintf(stdout, "%X ", this->canMsg.Data[i]);
    }
    fprintf(stdout, "...\n");
}

// void SkyWilling::CCUMsg::print() {
//     fprintf(stdout, "print ccumsg: \n");
//     for (int i = 0; i < 8; ++i) {
//         fprintf(stdout, "0x%X", this->packet[i]);
//     }
//     fprintf(stdout, "....\n");
// }

void VehicleStatusMsg::createMessage()
{
    this->rosMsg_vehicle_status.shift_level = shiftLevel();

    this->rosMsg_vehicle_status.cur_speed = speed();

    this->rosMsg_vehicle_status.cur_steer = wheelAngle();

    this->rosMsg_vehicle_status.wheel_direction = wheelDirection();

    this->rosMsg_vehicle_status.drive_mode = driveMode();

    this->rosMsg_vehicle_status.acc_level = accLevel();

    this->rosMsg_vehicle_status.brake_level = brakeLevel();

    this->rosMsg_vehicle_status.total_odometer = totalOdometer();
}

can_msgs::vehicle_status VehicleStatusMsg::getMessage()
{
    this->createMessage();
    return this->rosMsg_vehicle_status;
}

// -*-*-*-*- Received canMsg: battery_status -*-*-*-*-

double BatteryMsg::getVoltage()
{
    return readAsInt(this->canMsg.Data, BMU_VOLTAGE_OFFSET, BMU_VOLTAGE_LENGTH) * 0.1;
}

double BatteryMsg::getAmpere()
{
    return (readAsInt(this->canMsg.Data, BMU_AMPERE_OFFSET, BMU_AMPERE_LENGTH) - 4000) * 0.1;
}

double BatteryMsg::getBatteryCapacity()
{
    return readAsInt(this->canMsg.Data, BMU_CAPACITY_OFFSET, BMU_CAPACITY_LENGTH) * 0.4;
}

int BatteryMsg::getBsuSysStatus()
{
    return readAsInt(this->canMsg.Data, BMU_SYS_STATUS_OFFSET, BMU_SYS_STATUS_LENGTH);
}

int BatteryMsg::getChargeStatus()
{
    return readBit(this->canMsg.Data, BMU_CHARGE_STATUS_OFFSET);
}

void BatteryMsg::print()
{
    fprintf(stdout, "Recv BatteryMsg: "); // printf:格式化输出到屏幕stdout; fprintf格式化输出到文件FILE; sprintf格式化输出到字符创char*
    fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
    fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
    this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
    this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
    fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
    fprintf(stdout, "Data: ");
    for (int i = 0; i < 8; i++)
    {
        fprintf(stdout, "0x%X ", this->canMsg.Data[i]);
    }
    fprintf(stdout, "...\n");
}

void BatteryMsg::createMessage()
{
    this->rosMsg_battery_status.voltage = getVoltage();

    this->rosMsg_battery_status.ampere = getAmpere();

    this->rosMsg_battery_status.capacity = getBatteryCapacity();

    this->rosMsg_battery_status.BmuSys_status = getBsuSysStatus();

    this->rosMsg_battery_status.Charge_status = getChargeStatus();
}

can_msgs::battery BatteryMsg::getMessage()
{
    this->createMessage();
    return this->rosMsg_battery_status;
}
} // namespace USB2CAN