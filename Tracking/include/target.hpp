#pragma once
#include "defs.hpp"

namespace Target {
/*
 * 获取当前无人机的测试阶段。
 * PATROL: 巡逻阶段
 * OUTPOST: 前哨站阶段
 * BASE: 基地阶段
 */
Defs::DroneTarget GET_STAGE();
/*
 * 设置调试模式
 * mode = 0: 仅显示误差
 * mode = 1: 显示详细调试信息
 * mode = 2: 显示图形化界面
 * mode = 3: 两者都开启
 */
void ENABLE_DEBUG(Defs::DebugMode mode);
}  // namespace Target
