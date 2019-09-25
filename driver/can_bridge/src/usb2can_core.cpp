#include "usb2can_core.h"

namespace USB2CAN
{
void *receive_func(void *param_in)
{
    Param *param = (Param *)param_in;
    uint8_t mrun_num = param->mrun_num;
    std::string name = param->name;
    VCI_CAN_OBJ rec[3000]; // 接收缓存
    int reclen = 0;        // 获取收到的数据的帧数

    ros::NodeHandle nh_t;
    std::string battery_topic_;
    std::string vehicle_status_topic_;
    nh_t.param<std::string>("battery_topic", battery_topic_, "/battery");
    nh_t.param<std::string>("vehicle_status_topic", vehicle_status_topic_, "/vehicle_status");

    ros::Publisher pub_battery_status_ = param->battery_status_publisher;
    ros::Publisher pub_vehicle_sattus_ = param->vehicle_status_publisher;

    while (mrun_num & 0x0f)
    {
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100)) > 0) //设备类型,设备索引,can通道索引,接收缓存索引,接收缓存大小,WaitTime(保留参数)
        {
            for (int i = 0; i < reclen; i++)
            {
                if (rec[i].ID == 0x51) // vehicle status
                {
                    VehicleStatusMsg msg_factory(rec[i]);
                    can_msgs::vehicle_status msg_vehicle_status = msg_factory.getMessage();
                    msg_vehicle_status.Header.stamp = ros::Time::now();
                    pub_vehicle_sattus_.publish(msg_vehicle_status);
                }
                else if (rec[i].ID == 0x2AA) // battery status
                {
                    BatteryMsg msg_factory(rec[i]);
                    can_msgs::battery msg_battery_status = msg_factory.getMessage();
                    pub_battery_status_.publish(msg_battery_status);
                }
            }
        }
    }
    printf("[can_module] Listener-0 thread exit.\n");
    pthread_exit(0);
}

void *listen_can1(void *param_in)
{
    Param *param = (Param *)param_in;
    uint8_t mrun_num = param->mrun_num;
    VCI_CAN_OBJ recv[3000]; // 接收缓存
    int reclen = 0;         // 获取收到的数据的帧数
    while (mrun_num & 0x0f)
    {
        reclen = VCI_Receive(VCI_USBCAN2, 0, 1, recv, 3000, 100);
        if (reclen > 0)
        {
            for (int i = 0; i < reclen; i++)
            {
                if (recv[i].ID == 0x51) // vehicle status
                {
                    VehicleStatusMsg msg_factory(recv[i]);
                    msg_factory.print();
                }
                else if (recv[i].ID == 0x2AA) // battery status
                {
                    BatteryMsg msg_factory(recv[i]);
                    msg_factory.print();
                }
                else if (recv[i].ID == 0x120)
                {
                    fprintf(stdout, "Recv ecu Msg->");
                    SendMsg msg_factory(recv[i]);
                    msg_factory.print();
                }
            }
        }
    }
    printf("[can_module] Listener-1 thread exit.\n"); //退出接收线程
    pthread_exit(0);
}

CAN_app::CAN_app() : p_nh("~")
{
    initROS();

    // 打开CAN设备
    usleep(100000);
    if (VCI_OpenDevice(CAN_device, CAN_id, 0) == 1) // params: 设备类型,设备索引,预留位(通常为0)
    {
        ROS_INFO_STREAM("[can_module] open device success\n");
    }
    else
    {
        printf("[can module] open USB2CAN device failed once, try to ResetCAN...\n");
        if (VCI_ResetCAN(CAN_device, CAN_id, CAN_port) == 1) //复位CAN通道 //上一次can未正确关闭会导致打不开,需要重新"拔插"一次
        {
            printf("[can module] ResetCan success\n");
        }
        else
        {
            ROS_WARN_STREAM("[can_module] open device failed!\n");
            exit(1);
        }
    }

    config.AccCode = 0;          // 帧过滤验收码,详见说明文档及VCI_InitCAN  // 与AccMask共同决定哪些帧可以被接收
    config.AccMask = 0xFFFFFFFF; // 帧过滤屏蔽码,当前表示全部接收 TODO
    config.Filter = 2;           // 0/1 接收所有类型;2 只接收标准帧;3 只接收扩展帧
    config.Timing0 = 0x00;       // 这两个共同设置波特率,当前表示500k,其他请参照说明文档
    config.Timing1 = 0x1C;
    config.Mode = 0; // =0 正常模式; =1 只监听; =2 自发自收(回环模式)

    // 初始化CAN设备,CAN通道0
    usleep(100000);
    if (VCI_InitCAN(CAN_device, CAN_id, CAN_port, &config) != 1) // params: 设备类型, 设备索引, can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Init CAN-port-" << CAN_port);
        VCI_CloseDevice(CAN_device, CAN_id);
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Init CAN-port-" << CAN_port);
    }
    // 启动CAN设备, CAN通道0
    usleep(100000);
    if (VCI_StartCAN(CAN_device, CAN_id, CAN_port) != 1) // params: 设备类型,设备索引,can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Start CAN-port-" << CAN_port);
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Start CAN-port-" << CAN_port);
    }

    if (param_listen_can2)
    {
        // 初始化CAN设备,CAN通道1
        usleep(100000);
        if (VCI_InitCAN(CAN_device, CAN_id, CAN_port + 1, &config) != 1) // params: 设备类型, 设备索引, can通道编号
        {
            ROS_ERROR_STREAM("[can_module] ERROR Init CAN-port-" << CAN_port + 1);
            VCI_CloseDevice(CAN_device, CAN_id);
        }
        else
        {
            ROS_INFO_STREAM("[can_module] SUCCESS Init CAN-port-" << CAN_port + 1);
        }
        // 启动CAN设备, CAN通道1
        usleep(100000);
        if (VCI_StartCAN(CAN_device, CAN_id, CAN_port + 1) != 1) // params: 设备类型,设备索引,can通道编号
        {
            ROS_ERROR_STREAM("[can_module] ERROR Start CAN-port-" << CAN_port + 1);
            VCI_CloseDevice(VCI_USBCAN2, 0);
        }
        else
        {
            ROS_INFO_STREAM("[can_module] SUCCESS Start CAN-port-" << CAN_port + 1);
        }
    }
}
CAN_app::~CAN_app()
{
    printf("[can module] try to ResetCAN...");
    usleep(100000);                             //延时100ms。
    VCI_ResetCAN(CAN_device, CAN_id, CAN_port); //复位CAN通道。
    printf("[can module] ResetCan success");
    printf("[can module] try to CloseCAN...");
    usleep(100000);                  //延时100ms。
    VCI_CloseDevice(VCI_USBCAN2, 0); //关闭设备。
    printf("[can module] CloseCAN success! done");
}

void CAN_app::initROS()
{
    p_nh.param<bool>("if_listen_can2", param_listen_can2, false);
    int device_num;
    p_nh.param<int>("CAN_device_num", device_num, 2);
    if (device_num == 1)
    {
        CAN_device = VCI_USBCAN1;
    }
    else if (device_num == 2)
    {
        CAN_device = VCI_USBCAN2;
    }

    sub_ecu = nh.subscribe("ecu", 500, &CAN_app::ecu_cb, this);
    pub_vehicle_status = nh.advertise<can_msgs::vehicle_status>("vehicle_status", 500);
    pub_battery_status = nh.advertise<can_msgs::battery>("battery_status", 500);

    pre_steer = 0.;
}

void CAN_app::ecu_cb(const can_msgs::ecu::ConstPtr &msg)
{
    SendMsg sendMsg(*msg, pre_steer);
    pre_steer = msg->steer;
    VCI_CAN_OBJ sendData = sendMsg.getMessage();

    if (VCI_Transmit(CAN_device, CAN_id, CAN_port, &sendData, 1) != 1)
    {
        ROS_WARN_STREAM("[can_module] Transmit failed once.");
        ROS_INFO_STREAM("CAN_device : " << CAN_device);
        ROS_INFO_STREAM("CAN_id     : " << CAN_id);
        ROS_INFO_STREAM("CAN_port   : " << CAN_port);
        ROS_INFO_STREAM("sendData : ");
        sendMsg.print(); // 以十六进制的形式打印ecu->can消息
    }
    // else
    // {
    //     sendMsg.print();
    // }
}

class ThreadGuard
{
private:
    pthread_t &t1_;
    Param *param_;

public:
    explicit ThreadGuard(pthread_t &t1, Param *param) : t1_(t1), param_(param){};
    ~ThreadGuard()
    {
        if (t1_ != NULL)
        {
            param->mrun_num = 0;
            pthread_join(t1_, NULL);
        }
    }
    ThreadGuard(const ThreadGuard &) = delete;
    ThreadGuard &operator=(const ThreadGuard &) = delete;
};

void CAN_app::run()
{
    param = new Param();
    param->mrun_num = 1;
    param->name = "Listener Thread";
    param->battery_status_publisher = pub_battery_status;
    param->vehicle_status_publisher = pub_vehicle_status;
    int ret = pthread_create(&threadid, NULL, receive_func, param);
    ThreadGuard t1{threadid, param};

    pthread_t thread_listen_can1;
    if (param_listen_can2)
    {
        // 目前can通道1闲置,因此用来做监听,进行debug
        int ret1 = pthread_create(&thread_listen_can1, NULL, listen_can1, param);
        ThreadGuard t2{thread_listen_can1, param};
    }

    ros::spin();
    ros::shutdown();
}
};  // namespace USB2CAN
    // 写在最后: 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。