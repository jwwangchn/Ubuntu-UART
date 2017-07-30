#include "main.h"

using namespace std;
using namespace cv;
using namespace rs;


/*************************************************************************
*  函数名称：KylinBotMsgPullerThreadFunc
*  功能说明：数据接收线程相关函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017-07-27
×  TODO: 添加初始化
*************************************************************************/
void PullMsg()
{
    int rxLength = sizeof(rxMoveClawBoardMsg);
    uint8_t rxBuffMsg[rxLength];
    // 将结构体转换成字节数据
    int readLength = read_serial(rxBuffMsg, rxLength, TIMEOUT);
    if (readLength != 0)
    {
        memcpy(&rxMoveClawBoardMsg, rxBuffMsg, rxLength);
    }
}


/*************************************************************************
*  函数名称：KylinBotMsgPusherThreadFunc
*  功能说明：数据发送线程相关函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017-07-27
×  TODO: 添加初始化
*************************************************************************/
void PushMsg()
{
    int txLength = sizeof(txMoveClawBoardMsg);
    uint8_t txBuffMsg[txLength];
    // 将结构体转换成字节数据
    memcpy(txBuffMsg, &txMoveClawBoardMsg, txLength);
    write_serial(txBuffMsg, txLength, TIMEOUT);
}

/*************************************************************************
*  函数名称：testUARTFun
*  功能说明：测试数据收发的函数
*  参数说明：输入写入的数据
*  函数返回：无
*  修改时间：2017-07-28
*  TODO: 添加函数体
*************************************************************************/
int testUARTFun()
{
    const char *device = "/dev/ttyUSB0";
    if (connect_serial(device, 115200) == -1)
    {
        printf("serial open error!\n");
        return -1;
    }
    while (true)
    {
        unsigned char txBuff[2] = {'A','B'};
        cout << write_serial(txBuff, sizeof(txBuff), TIMEOUT) << endl;

        // unsigned char rxBuff;
        // read_serial(&rxBuff, sizeof(rxBuff), TIMEOUT);
        // cout << "rxBuff: "<<rxBuff<<endl;

        // typedef struct dataStruct
        // {
        //     uint16_t x;
        //     uint16_t y;
        // } dataStruct;
        // dataStruct data;

        // int rxLength = sizeof(data);
        // uint8_t rxBuffMsg[rxLength];
        // // 将结构体转换成字节数据
        // int readLength = read_serial(rxBuffMsg, rxLength, TIMEOUT);
        // if (readLength != 0)
        // {
        //     memcpy(&data, rxBuffMsg, rxLength);
        //     cout << data.x << " " << data.y << endl;
        // }

        sleep(1);
    }
}

int main()
{
    // 计时函数
    missionStartTimeUs = currentTimeUs();

    signal(SIGINT, sigfunc); // 设置信号, 对 ctrl+C 进行处理, 终止程序

    // 相机相关参数设定
#ifdef _USE_LIGHT_CAMERA
    if (!setcamera())
    {
        cout << "Setup camera failure. Won't do anything." << endl;
        return -1;
    }
#endif
    // 打开 RealSense
#ifdef _USE_REALSENSE
    rs::log_to_console(rs::log_severity::warn);

    if (!initialize_streaming())
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console(rs::log_severity::fatal);
        return EXIT_FAILURE;
    }
    // setup_windows();
#endif

    cout << sizeof(txMoveClawBoardMsg) << endl;
    cout << sizeof(rxMoveClawBoardMsg) << endl;

    // 系统初始化
    init();

    // 打开串口
    /*
    const char *device = "/dev/ttyTHS2";
    if (connect_serial(device, 115200) == -1)
    {
        printf("serial open error!\n");
        return -1;
    }
    */
    // 测试串口
    return (testUARTFun());

    // 创建线程
    MyThread kylibotMsgPullerTread;     //数据读取
    MyThread kylibotMsgPusherTread;     //数据发送
    MyThread kylibotMarkDetectionTread; //检测线程

    kylibotMsgPullerTread.create(KylinBotMsgPullerThreadFunc, NULL);
    kylibotMsgPusherTread.create(KylinBotMsgPusherThreadFunc, NULL);
    kylibotMarkDetectionTread.create(KylinBotMarkDetecThreadFunc, NULL);

    // 测试视觉处理过程专用, 1->矩形检测 2->箭头检测, 其他->跳过
    testVisionProcessFun(1);

    //主逻辑循环
    while (!exit_flag)
    {
        // workState: 0->
        switch (workState)
        {
        case 1:
            // 1. 矩形引导小车前进，直到 realsense 检测到的距离小于多少时, 切换到 case2
            detection_mode = 1; // 矩形检测
            coutLogicFlag = 1;
            workStageCout = "阶段 1 : ";
            workStateCout = "矩形引导小车抓盒子";

            // 发送小车前进指令
            // TODO: 根据底层数据格式修改单位, 不能忘了！！！！
            txMoveMsg(tx, X_SPEED, tz, Y_SPEED, ry, Z_SPEED);
            txClawBoardMsg(0, 0, 0, 0);

            // TODO: 根据线程中的标志位切换来修改 if 语句, 对照之前的程序
            if (tx < DISTANCE_RECTANGLE_TO_ARROW)
            {
                workState = 2;
                detection_mode = 2;
            }
            break;
        case 2:
            // 2. 箭头引导小车前进，直到盒子完全进入小车(如何判断)
            detection_mode = 2; // 箭头检测
            coutLogicFlag = 2;
            workStageCout = "阶段 2 : ";
            workStateCout = "箭头引导小车抓盒子";

            // 发送小车前进指令
            // TODO: 根据底层数据格式修改单位, 不能忘了！！！！
            // TODO: 确认坐标系
            txMoveMsg(tx, X_SPEED, tz, Y_SPEED, ry, Z_SPEED);
            txClawBoardMsg(0, 0, 0, 0);
            // TODO: 根据线程中的标志位切换来修改 if 语句, 对照之前的程序
            if (tx < DISTANCE_RECTANGLE_TO_ARROW)
            {
                workState = 3;
                detection_mode = 0;
            }
            break;
        case 3:
            // 3. 抓取盒子
            detection_mode = 0; // 关闭检测程序
            coutLogicFlag = 3;
            workStageCout = "阶段 3 : ";
            workStateCout = "抓取盒子";
            // TODO: 抓取函数
            // 小车静止不动
            txMoveMsg(0, 0, 0, 0, 0, 0);
            // TODO: 爪子应该是先合拢, 再升高, 挡板的速度需要再修改
            txClawBoardMsg(0, CLAW_SPEED, 0, BOARD_SPEED);
            // TODO: 根据爪子升高的位置来修改 if 语句
            if (rxMoveClawBoardMsg.positionClaw > CLAW_HEIGHT)
            {
                // TODO: 添加校准函数的位置
                workState = 4;
                detection_mode = 0;
            }
            break;
        case 4:
            // 4. 后退 2m, 绝对位置控制
            detection_mode = 0; // 关闭检测程序
            coutLogicFlag = 4;
            workStageCout = "阶段 4 : ";
            workStateCout = "小车后退约 2m , 使小车左移能到达基地区";
            // 发送小车前进指令
            // TODO: 根据底层数据格式修改单位, 不能忘了！！！！
            // TODO: 修改 DISTANCE_BACKWARDS
            txMoveMsg(0, X_SPEED, 0, Y_SPEED, 0, 0);
            // 保持爪子不动
            txClawBoardMsg(0, 0, 0, 0);
            // TODO: 到达指定位置, 满足 if 条件, 注意修改 if 条件
            if (1)
            {
                workState = 5;
                detection_mode = 0;
            }
            break;
        case 5:
            // 5. 向左移动一段距离, 绝对位置控制
            detection_mode = 0; // 关闭检测程序
            coutLogicFlag = 5;
            workStageCout = "阶段 5 : ";
            workStateCout = "小车向左移动一定距离 , 到达基地区边缘";
            // 发送小车前进指令
            // TODO: 根据底层数据格式修改单位, 不能忘了！！！！
            // TODO: 修改 DISTANCE_LEFTWARDS
            txMoveMsg(0, X_SPEED, 0, Y_SPEED, 0, 0);
            // 保持爪子不动
            txClawBoardMsg(0, 0, 0, 0);
            // TODO: 到达指定位置, 满足 if 条件, 注意修改 if 条件
            if (1)
            {
                workState = 6;
                detection_mode = 2;
            }
            break;
        case 6:
            // 6. 箭头检测定位基地区
            detection_mode = 2; // 箭头检测
            coutLogicFlag = 6;
            workStageCout = "阶段 6 : ";
            workStateCout = "箭头引导小车放盒子";

            // 发送小车前进指令
            // TODO: 根据底层数据格式修改单位, 不能忘了！！！！
            txMoveMsg(tx, X_SPEED, tz, Y_SPEED, ry, Z_SPEED);
            txClawBoardMsg(0, 0, 0, 0);
            // TODO: 根据线程中的标志位切换来修改 if 语句, 对照之前的程序
            if (tx < DISTANCE_RECTANGLE_TO_ARROW)
            {
                workState = 7;
                detection_mode = 0;
            }
            break;
        case 7:
            // 7. 小车过障碍
            detection_mode = 0; // 箭头检测
            coutLogicFlag = 7;
            workStageCout = "阶段 7 : ";
            workStateCout = "过障碍";
            // TODO: 根据底层程序修改
            txMoveClawBoardMsg.flags = flagsSet(txMoveClawBoardMsg.flags, CONTROL_PASS_OBSTACLE_BIT, 1);
            if (flagsRead(rxMoveClawBoardMsg.flags, READ_PASS_OBSTACLE_BIT))
            {
                workState = 8;
                detection_mode = 0;
            }
            break;
        case 8:
            // 8. 堆叠盒子
            coutLogicFlag = 8;
            workStageCout = "阶段 8 : ";
            workStateCout = "堆叠盒子";
            // TODO: 封装成一个函数
            if (pileBoxes())
            {
                workState = 9;
                detection_mode = 0;
            }

            break;
        case 9:
            // 9. 小车过障碍
            detection_mode = 0; // 箭头检测
            coutLogicFlag = 9;
            workStageCout = "阶段 9 : ";
            workStateCout = "过障碍";
            // TODO: 根据底层程序修改
            txMoveClawBoardMsg.flags = flagsSet(txMoveClawBoardMsg.flags, CONTROL_PASS_OBSTACLE_BIT, 1);
            if (flagsRead(rxMoveClawBoardMsg.flags, READ_PASS_OBSTACLE_BIT))
            {
                // TODO: 添加校准函数的位置
                workState = 10;
                detection_mode = 0;
            }
            break;
        case 10:
            // 10. 小车向右移动一段距离
            detection_mode = 0; // 关闭检测程序
            coutLogicFlag = 10;
            workStageCout = "阶段 10 : ";
            workStateCout = "小车向右移动一定距离 , 准备进行下一次抓取";
            // 发送小车前进指令
            // TODO: 根据底层数据格式修改单位, 不能忘了！！！！
            // TODO: 修改 DISTANCE_LEFTWARDS
            txMoveMsg(0, X_SPEED, 0, Y_SPEED, 0, 0);
            // 保持爪子不动
            txClawBoardMsg(0, 0, 0, 0);
            // TODO: 到达指定位置, 满足 if 条件, 注意修改 if 条件
            if (1)
            {
                workState = 1;
                detection_mode = 2;
                boxNum++;
            }
            break;
        default:
            break;
        }
        if (boxNum > MAX_GRASP_NUM)
        {
            exit_flag = 1; //完成任务
            break;
        }
    }
    missionEndTimeUs = currentTimeUs();
    cout << "任务用时: " << (missionEndTimeUs - missionStartTimeUs) * 1e-6 << endl;
    capture.closeStream(); // 关闭摄像头
    disconnect_serial();   // 关闭串口
    cout << "任务完成, 关闭程序!!!" << endl;
    return 0;
}

/*************************************************************************
*  函数名称：graspBoxes
*  功能说明：抓取盒子逻辑过程
*  参数说明：无
*  函数返回：无
*  修改时间：2017-07-28
*  TODO: 添加函数体
*************************************************************************/
bool graspBoxes()
{
    bool result = false;
    return result;
}

/*************************************************************************
*  函数名称：pileBoxes
*  功能说明：堆叠盒子逻辑过程
*  参数说明：无
*  函数返回：无
*  修改时间：2017-07-28
*  TODO: 添加函数体
*************************************************************************/
bool pileBoxes()
{
    bool result = false;
    return result;
}

/* 急需确认项
TODO: TODO: TODO:

1. 小车坐标系的定义
2. 前进距离的单位 速度的单位 旋转角度的单位
3. 通信协议
4. 速度为 0 时, 保持当前状态不变
5. 绝对位置控制, 如何判断到达了指定位置? 以前使用 当前点和目标点的error作为条件
6. 绝对位置和相对位置如何切换
7. 小车过障碍在底层完成, 上层发送指令, 下层完成之后, 返回一个指令
*/

/* 逻辑流程

1. 矩形引导小车前进，直到realsense检测到的距离小于
2. 箭头引导小车前进，直到盒子完全进入小车(如何判断)
3. 抓取盒子
4. 后退 2m, 绝对位置控制
5. 向左移动一段距离, 绝对位置控制
6. 箭头检测定位基地区
7. 小车过障碍
8. 堆叠盒子
9. 小车过障碍
10. 小车向右移动一段距离

*/

/* 通信协议

1. 小车 x y 方向的移动距离和速度
2. 切换绝对位置和相对位置的标志位
3. 滑台的位置和速度
4. 过障碍的标志位
5. 推板的位置和速度

*/

/* 视觉引导程序

1. 矩形检测 旋转量 前后 左右偏移量
2. 箭头检测 左右偏移量 质心位置 质心距离 realsense 的距离

*/

/* 里程计校准

1. 抓取盒子时，进行校准，记录盒子个数
2. 放完盒子之后，进行校准

*/