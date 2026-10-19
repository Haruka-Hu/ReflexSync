#include <Arduino.h>

#include "master_node.h"

namespace {

// Master 节点：本地采 6x ADS1298 48CH EMG，同时接收 IMU Slave 的 ESP-NOW batch。
// ADS 引脚：CS1..CS6, MOSI, MISO, SCK, DRDY, RESET_N, START。
MasterApp g_app({
    {{16, 15, 7, 6, 5, 4}, 11, 13, 12, 9, 14, 17},
    4000000,
});

}  // namespace

// Arduino 启动入口：把 Master 应用初始化并交给 FreeRTOS 任务运行。
void setup() {
  g_app.begin();
}

// Arduino 主循环：Master 无周期逻辑，永久挂起以节省调度开销。
void loop() {
  // MasterApp 已经创建 FreeRTOS 任务，Arduino loop 不再承担周期逻辑。
  vTaskDelay(portMAX_DELAY);
}
