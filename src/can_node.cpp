// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"
std::shared_ptr<Tangair_usb2can> CAN_ptr;

void signal_callback_handler(int signum){
    CAN_ptr->~Tangair_usb2can();
}
/*
主函数
一个主函数循环、一个接收线程
*/
int main()
{
         /* set real-time process */
        pid_t pid = getpid();
        sched_param param;
         param.sched_priority = sched_get_priority_max(SCHED_FIFO);
         if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }

   //创建对象，调用构造函数
   //开启订阅，设置回调函数，创建线程，在线程接收LCM数据
   CAN_ptr = std::make_shared< Tangair_usb2can >();

//   //设置信号中断
    signal( SIGINT,  signal_callback_handler);

   //主函数循环，1s延时
    CAN_ptr->Spin();

//    
    return 0;
}

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0

