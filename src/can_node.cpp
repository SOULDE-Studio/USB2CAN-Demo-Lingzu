// ================= 选择运行模式 =================

// 调用Zbot_RL类的编译步骤(根目录下执行)
// rm -rf build
// mkdir build
// cd build
// cmake ..
// make -j
// 已写成脚本build_Zbot_RL.sh，直接运行脚本即可

// 调用Motor_test类的编译步骤(根目录下执行)
// rm -rf build
// mkdir build
// cd build
// cmake -DUSE_MOTOR_TEST=ON ..
// make -j
// 已写成脚本Motor_test.sh，直接运行脚本即可

// =================================================

#ifdef USE_MOTOR_TEST
    #include "Motor_test.h"
    using AppType = Motor_test;
#else
    #include "Zbot_RL.h"
    using AppType = Zbot_RL;
#endif

std::shared_ptr<AppType> App_ptr;

void signal_callback_handler(int)
{
    App_ptr->~AppType(); 
}

int main()
{
    /* real-time scheduler */
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(pid, SCHED_FIFO, &param);

    App_ptr = std::make_shared<AppType>();

    signal(SIGINT, signal_callback_handler);

    App_ptr->Spin();

    return 0;
}


