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
/* Internal INIT result gate, not the writable A2L status/mode mirrors. */
bool MainInt_InitBlocksOutput(void);
extern volatile DeviceStateEnum_t MainInt_State;
bool MainInt_ModeRequestsStop(void);
void MainInt_PrepareMode(void);
#endif
void MainInt_ControlBackground(void);

#endif /* __MAIN_INT_H__ */
