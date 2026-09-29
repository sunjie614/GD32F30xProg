#ifndef __MAIN_INT_H__
#define __MAIN_INT_H__

#include <stdbool.h>

typedef enum {
    INIT,        // 基础初始化：仅获取系统参数
    RUNNING,     // 运行：正常工作状态
    SENSORLESS,  // 传感器无位置模式
    EXIT
} DeviceStateEnum_t;

void Main_Int_Handler(void);
void MainInt_ControlInit(void);
#if defined(MC_NUMERIC_IQMATH)
bool MainInt_ControlReady(void);
#endif
void MainInt_ControlBackground(void);

#endif /* __MAIN_INT_H__ */
