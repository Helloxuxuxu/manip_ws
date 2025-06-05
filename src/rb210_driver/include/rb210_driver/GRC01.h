/**
 * @file GRC01.h
 * @author wenyu (785895681@qq.com)
 * @brief  自定义的GRC01 API，用于模拟控制，后续需要向广数请求这个库函数，用于替换。
 * @version 0.1
 * @date 2025-04-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */


 
 // 1.网络接口
/*----------------------------模块启动函数---------------------------*/
/**
* @brief 网络模块启动函数
* @return 启动并连接成功返回 true，否则 false* 
错误码，0 表示正常,其他表示错误，具体参考返回值列表
*/
/*-----------------------------------------------------------------*/
short GRC01_Run(){};
/*----------------------------模块终止函数---------------------------*/
/**
* @brief 网络模块终止函数
* @return 无
[注意]
此操作不会释放该网络模块对象所占资源
* @return 
错误码，0 表示正常,其他表示错误，具体参考返回值列表
*/
/*-----------------------------------------------------------------*/
short GRC01_Stop(){};
/*---------------------------客户端添加函数--------------------------*/
/**
* @brief  创建/移除/退出客户端连接
* @param  serIp
控制器的 IP
* @param  serPort
控制器的端口号
* @param  serPortAcq
控制器采集功能端口号
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
*/
/*-----------------------------------------------------------------*/
short GRC01_CreateClient(char *serIp, unsigned short serPort,
  unsigned short serPortAcq){};






// 读取系统运动状态
/*--------------------------机器人位姿值获取--------------------------*/
/**
* @brief 获取机器人位姿值
* @param cliSocket 用于传输的客户端，cosnt int 类型的引用
* @param flag 是否重新获取，true 重新获取，false 只返回数值
* @param count 获取到的有效位姿值个数,int 类型引用
* @param pa[6] 获取到的位姿值,double 数组类型,依次为 X、Y、Z、W、P、R 值
(X/Y/Z:mm,W/P/R:deg),失败时返回容器实际元素个数为 0
* @return 
错误码，0 表示正常,其他表示错误，具体参考返回值列表
*/
/*-----------------------------------------------------------------*/
short GRC01_GetDcarActPos(const int& cliSocket, bool flag, int &count, double pa[6]){};
/*--------------------------机器人关节值获取--------------------------*/
/**
* @brief 获取机器人关节值
* @param cliSocket
* @param flag 是否重新获取，true 重新获取，false 只返回数值
* @param count 有效关节值个数，int 类型引用
* @param pa[6] 获取到的关节值，double 类型数组,依次为 S、L、U、R、B、T 值(deg)
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于传输的客户端，cosnt int 类型的引用
*/
/*-----------------------------------------------------------------*/
short GRC01_GetJointActPos(const int& cliSocket, bool flag, int &count, double pa[6]){};
/*--------------------------获取系统运行状态--------------------------*/
/**
* @brief 获取机器人的获取系统运行状态
* @param cliSocket 用于传输的客户端，cosnt int 类型的引用
* @param flag 是否重新获取，true 重新获取，false 只返回数值
* @param val 获取到的系统运行状态,unsigned short 类型引用,0:停止 1:暂停 2:急停 3:运行
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
*/
/*-----------------------------------------------------------------*/
short GRC01_GetSysRunState(const int& cliSocket, bool flag, unsigned short &val){};
/*-------------------------获取机器人的使能状态------------------------*/
/**
* @brief  获取机器人当前的使能状态
* @param  cliSocket 用于传输的客户端，cosnt int 类型的引用
* @param  flag 是否重新获取，true 重新获取，false 只返回数值
* @param  val 获取到的使能状态,unsigned short 类型引用
* @return  错误码，0 表示正常,其他表示错误，具体参考返回值列表
*/
/*------------------------------------------------------------------*/
short GRC01_GetintCanUse(const int& cliSocket, bool flag, unsigned short &val){};






// 运动控制相关函数
// 1. 直线运动
/*---------------------设置直线运动速度----------------------------*/
/**
* @brief 设置直线运动速度
* @param cliSocket
* @param vel
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
直线运动速度,单位 mm/s
*/
short GRC01_SetLineSpeed(const int &cliSocket, double vel){};
/*-------------------------直线运动----------------------------*/
/**
* @brief  控制机器人直线运动
* @param  x,y,z,Rx,Ry,Rz,j7,j8 机器人位置姿态坐标和外部轴角度
* @param  ismodel
* @return  错误码，0 表示正常,其他表示错误，具体参考返回值列表
接口是否阻塞
*/
/*-------------------------------------------------------------------*/
short GRC01_MoveL(const int &cliSocket, double x, double y, double z, double Rx, double Ry, double Rz, double
  j7, double j8,int ismodel = 0){return 0;};

// 2. 关节运动
/*-------------------设置关节运动速度比率----------------------------*/
/**
* @brief 设置关节运动速度百分比
* @param cliSocket
* @param ratio
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
关节运动速度百分比[1%-100%]
*/
short GRC01_SetJointSpeedRatio(const int &cliSocket, short ratio){};

/*---------------------关节运动(关节角度)-----------------------*/
/**
* @brief 控制机器人关节运动(使用关节角度值)
* @param cliSocket
* @param j1~j8 关节终点角度值
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
*/
short GRC01_MoveJ(const int &cliSocket, double j1, double j2, double j3, double j4, double j5, double j6,
  double j7, double j8, int ismodel = 0){return 0;};

/*----------------------关节运动(笛卡尔)-----------------------*/
/*** @brief 控制机器人关节运动(使用笛卡尔坐标)
* @param x,y,z,Rx,Ry,Rz,j7,j8 机器人位置姿态坐标和外部轴角度
* @param ismodel
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
接口是否阻塞
*/
short GRC01_MoveJD(const int &cliSocket, double x, double y, double z, double Rx, double Ry, double Rz, double
  j7, double j8, int ismodel = 0){};


// 3. 路径运动
/*----------------------清除路径缓冲--------------------------*/
/**
* @brief 清除路径缓冲
* @param cliSocket
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
*/
short GRC01_ClearPath(const int &cliSocket){};

/*-------------------------添加路径----------------------------*/
/**
* @brief 添加路径
* @param cliSocket
* @param x,y,z,Rx,Ry,Rz,o1,o2 机器人位置姿态坐标和外部轴角度
* @param vel 路径运动速度
* @param zone 过渡值[0-8]
* @param type 运动方式,0:MOVJ(关节) 1:MOVJD(笛卡尔关节) 2:MOVL(直线) 3:MOVC(圆弧)
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
*/
short GRC01_AddMovePath(const int &cliSocket, double x, double y, double z, double Rx, double Ry, double
  Rz, double j7, double j8, double vel, short zone, short type = 0){};

/*-------------------获取路径缓冲剩余空间--------------------*/
/**
* @brief 获取路径缓冲区剩余空间
* @param cliSocket
* @param space 路径缓冲区剩余空间，引用类型，缓冲区最大 1000 条。
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
*/
short GRC01_GetPathSpace(const int &cliSocket, unsigned int &space){};

/*----------------------路径缓冲开始运动--------------------------*/
/**
* @brief 路径缓冲运动开始
* @param cliSocket
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
*/
short GRC01_RunPath(const int &cliSocket){};

/*-------------------获取路径缓冲运动状态--------------------------*/
/**
* @brief 获取路径缓冲运动状态
* @param cliSocket
* @param state 路径运动状态,0：空闲 1:运动中 2:暂停中
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
用于采集的客户端，cosnt int 类型的引用
*/
short GRC01_GetRunPathState(const int &cliSocket, short &state){state = 1;};



// 参考API列表，模拟按键输入
/*----------------------------模拟按键输入---------------------------*/
/**
* @brief 下发键值，模拟按键输入
* @param cliSocket
用于传输的客户端，cosnt int 类型的引用
* @param keyval
需要下发的键值，unsigned char 类型
* @return 错误码，0 表示正常,其他表示错误，具体参考返回值列表
[注意]
相关键值请查询说明文档
*/
/*-------------------------------------------------------------------*/
short GRC01_SimulateKeyInput(const int &cliSocket, unsigned char val){};


