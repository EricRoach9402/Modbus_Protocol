# Modbus TCP Server 使用說明

`modbus_tcp_server.h` 提供一套 Callback 式的 Modbus TCP Server（Slave）API，讓應用層只需實作讀寫回呼函式，即可透過 TCP 對外提供 Modbus 服務，無需處理底層 Socket 或 Modbus 封包格式。

---

## 目錄

1. [快速開始](#1-快速開始)
2. [引入標頭檔](#2-引入標頭檔)
3. [支援的功能碼](#3-支援的功能碼)
4. [定義暫存器地圖](#4-定義暫存器地圖)
5. [實作讀取回呼（on_read）](#5-實作讀取回呼on_read)
6. [實作寫入回呼（on_write）](#6-實作寫入回呼on_write)
7. [設定與啟動](#7-設定與啟動)
8. [連線事件通知（on_link，可選）](#8-連線事件通知on_link可選)
9. [停止 Server](#9-停止-server)
10. [日誌（可選）](#10-日誌可選)
11. [完整範例](#11-完整範例)
12. [注意事項](#12-注意事項)

---

## 1. 快速開始

```c
static int my_read(uint8_t fc, uint16_t addr, uint16_t qty,
                   uint16_t *out, void *userdata) {
    for (uint16_t i = 0; i < qty; i++) out[i] = 0;
    return 0;
}
static int my_write(uint16_t addr, uint16_t qty,
                    const uint16_t *data, void *userdata) {
    return 0;
}

mb_tcp_server_ctx_t    server = {0};
mb_tcp_server_config_t cfg;

mb_tcp_server_config_init(&cfg);  // 填入所有可選欄位的安全預設值
cfg.port     = 502;
cfg.unit_id  = 1;
cfg.on_read  = my_read;
cfg.on_write = my_write;

mb_tcp_server_start(&server, &cfg);

// 主迴圈 ...

mb_tcp_server_stop(&server);
```

---

## 2. 引入標頭檔

```c
#include "modbus_tcp_server.h"
```

---

## 3. 支援的功能碼

| 功能碼 | 說明 | 觸發回呼 |
|--------|------|---------|
| FC03 – Read Holding Registers | 讀取 Holding Registers | `on_read`（`function_code = 0x03`） |
| FC04 – Read Input Registers | 讀取 Input Registers | `on_read`（`function_code = 0x04`） |
| FC06 – Write Single Register | 寫入單一暫存器 | `on_write`（`qty` 固定為 1） |
| FC16 – Write Multiple Registers | 寫入多個暫存器 | `on_write` |

其他功能碼（FC01、FC02、FC05 等）的請求會收到 `Illegal Function` 例外回應，不會觸發任何回呼。

---

## 4. 定義暫存器地圖

建議以具名常數定義暫存器地址與總大小，方便維護。所有地址均為 **0-based**：

```c
// R   = 唯讀（僅由 on_read 回呼提供值）
// R/W = 可讀可寫（on_read 提供，on_write 接收）

#define REG_TEMPERATURE   0x00   // R   – 溫度 × 10（253 = 25.3 °C）
#define REG_HUMIDITY      0x01   // R   – 相對濕度 %
#define REG_STATUS        0x02   // R   – 狀態 bitmask
#define REG_UPTIME_LOW    0x03   // R   – 運行秒數（低 16 bit）
#define REG_UPTIME_HIGH   0x04   // R   – 運行秒數（高 16 bit）
#define REG_SETPOINT      0x0A   // R/W – 目標設定值
#define REG_CONTROL       0x0B   // R/W – 控制旗標
#define REG_DEVICE_ID     0x14   // R   – 裝置識別碼

// 暫存器總數；所有超出此範圍的請求應拒絕
#define REG_BANK_SIZE     32
```

---

## 5. 實作讀取回呼（on_read）

當 Master 發送 FC03 或 FC04 時，Server 會呼叫此函式。

**函式簽章：**

```c
typedef int (*mb_srv_read_fn)(
    uint8_t   function_code, // MODBUS_FUNC_READ_HOLDING_REGISTERS (0x03)
                             // 或 MODBUS_FUNC_READ_INPUT_REGISTERS (0x04)
    uint16_t  addr,          // 起始暫存器地址（0-based）
    uint16_t  qty,           // 請求讀取的數量（1 ~ 125）
    uint16_t *out,           // 填入 qty 個值（Host byte order）
    void     *userdata       // 對應 cfg.userdata
);
```

**回傳值：**

| 回傳值 | 效果 |
|--------|------|
| `0` | 成功，`out` 中的資料回傳給 Master |
| `MODBUS_EX_ILLEGAL_DATA_ADDRESS` (2) | 回傳 Exception 02（地址超出範圍） |
| `MODBUS_EX_ILLEGAL_DATA_VALUE` (3) | 回傳 Exception 03（數值不合法） |
| `MODBUS_EX_SERVER_DEVICE_FAILURE` (4) 或其他非零值 | 回傳 Exception 04（裝置內部錯誤） |

**`function_code` 的用途：**

FC03（Holding Registers）與 FC04（Input Registers）在 Modbus 語義上是不同的位址空間。如果裝置需要區分兩者，可依 `function_code` 分開處理；若不需要，忽略即可：

```c
static int on_read(uint8_t function_code, uint16_t addr, uint16_t qty,
                   uint16_t *out, void *userdata)
{
    // 區分 FC03 / FC04 的範例：
    if (function_code == MODBUS_FUNC_READ_INPUT_REGISTERS) {
        // 從硬體輸入暫存器空間讀取
    } else {
        // 從 Holding Register 空間讀取
    }

    // 若不需要區分，直接忽略 function_code：
    (void)function_code;
    // ...
}
```

**完整實作範例：**

```c
static uint16_t        reg_bank[REG_BANK_SIZE];
static pthread_mutex_t reg_lock = PTHREAD_MUTEX_INITIALIZER;

static int on_read(uint8_t function_code, uint16_t addr, uint16_t qty,
                   uint16_t *out, void *userdata)
{
    (void)userdata;
    (void)function_code; // 本範例兩個空間共用同一暫存器陣列

    // 1. 驗證地址範圍（使用 uint32_t 防止 addr + qty 整數溢位）
    if ((uint32_t)addr + (uint32_t)qty > REG_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    // 2. 加鎖，更新即時數值後回傳
    pthread_mutex_lock(&reg_lock);

    reg_bank[REG_TEMPERATURE] = 253;       // 25.3 °C
    reg_bank[REG_HUMIDITY]    = 60;
    reg_bank[REG_STATUS]      = 0x0001;

    uint32_t uptime = (uint32_t)time(NULL);
    reg_bank[REG_UPTIME_LOW]  = (uint16_t)(uptime & 0xFFFF);
    reg_bank[REG_UPTIME_HIGH] = (uint16_t)(uptime >> 16);
    reg_bank[REG_DEVICE_ID]   = 0x0100;

    for (uint16_t i = 0; i < qty; i++) {
        out[i] = reg_bank[addr + i];
    }

    pthread_mutex_unlock(&reg_lock);
    return 0;
}
```

> **若 `on_read` 設為 `NULL`**，Server 對所有讀取請求自動回應 Exception 04。

---

## 6. 實作寫入回呼（on_write）

當 Master 發送 FC06 或 FC16 時，Server 會呼叫此函式。

**函式簽章：**

```c
typedef int (*mb_srv_write_fn)(
    uint16_t        addr,      // 起始暫存器地址（0-based）
    uint16_t        qty,       // 寫入數量（FC06 固定為 1）
    const uint16_t *data,      // 要寫入的值陣列（Host byte order）
    void           *userdata   // 對應 cfg.userdata
);
```

**回傳值與 `on_read` 相同：**

| 回傳值 | 效果 |
|--------|------|
| `0` | 成功 |
| `MODBUS_EX_ILLEGAL_DATA_ADDRESS` (2) | 回傳 Exception 02 |
| `MODBUS_EX_ILLEGAL_DATA_VALUE` (3) | 回傳 Exception 03 |
| 其他非零值 | 回傳 Exception 04 |

**完整實作範例：**

```c
static int on_write(uint16_t addr, uint16_t qty,
                    const uint16_t *data, void *userdata)
{
    (void)userdata;

    // 1. 驗證地址範圍
    if ((uint32_t)addr + (uint32_t)qty > REG_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    // 2. 加鎖後寫入暫存器陣列
    pthread_mutex_lock(&reg_lock);
    for (uint16_t i = 0; i < qty; i++) {
        reg_bank[addr + i] = data[i];
    }
    pthread_mutex_unlock(&reg_lock);

    // 3. 針對特定暫存器執行對應動作
    for (uint16_t i = 0; i < qty; i++) {
        switch ((uint16_t)(addr + i)) {
            case REG_SETPOINT:
                printf("Setpoint → %u\n", data[i]);
                // 在此驅動硬體輸出
                break;
            case REG_CONTROL:
                printf("Control → 0x%04X\n", data[i]);
                // 在此解析控制旗標
                break;
            default:
                break;
        }
    }
    return 0;
}
```

> **若 `on_write` 設為 `NULL`**，Server 對所有寫入請求自動回應 Exception 04。

---

## 7. 設定與啟動

### 7.1 宣告 Context

```c
mb_tcp_server_ctx_t server = {0};  // 必須以 {0} 或 memset 零初始化
```

### 7.2 初始化設定

使用 `mb_tcp_server_config_init()` 初始化設定結構，它會將所有可選欄位填入安全預設值。之後只需設定必要欄位以及需要覆寫的選項：

```c
mb_tcp_server_config_t cfg;
mb_tcp_server_config_init(&cfg);  // 填入所有可選欄位的安全預設值

// 必填：協定關鍵欄位
cfg.port    = 502;
cfg.unit_id = 1;

// 必填（有意義的服務）：讀寫回呼
cfg.on_read  = on_read;
cfg.on_write = on_write;
cfg.userdata = &my_app_ctx;  // 若無需傳遞自訂資料可設 NULL

// 可選：僅在需要覆寫預設值時才設定
// cfg.max_clients    = 3;       // 預設：MB_TCP_MAX_CLIENTS（10）
// cfg.recv_timeout_ms = 10000;  // 預設：5000 ms
// cfg.logv           = my_logv; // 預設：NULL（靜默）
// cfg.on_link        = my_link; // 預設：NULL（不通知）
```

**`mb_tcp_server_config_init()` 預填的預設值：**

| 欄位 | 預設值 | 說明 |
|------|--------|------|
| `recv_timeout_ms` | 5000 ms | 每個 Client Socket 的接收逾時 |
| `max_clients` | 0 → `MB_TCP_MAX_CLIENTS`（10） | 最大同時連線數 |
| `on_read` | `NULL` | FC03/FC04 回應 Exception 04 |
| `on_write` | `NULL` | FC06/FC16 回應 Exception 04 |
| `userdata` | `NULL` | — |
| `logv` | `NULL` | 靜默模式 |
| `on_link` | `NULL` | 不通知連線事件 |

**關於 `unit_id`：**

```c
cfg.unit_id = 1;                        // 只接受 Unit ID = 1，其他靜默忽略
cfg.unit_id = MB_TCP_SERVER_UNIT_ID_ANY; // 接受所有 Unit ID（值為 0xFF）
```

### 7.3 啟動 Server

```c
if (mb_tcp_server_start(&server, &cfg) != 0) {
    fprintf(stderr, "Server 啟動失敗（Port %u）\n", cfg.port);
    return -1;
}
printf("Modbus TCP Server 監聽 Port %u（Unit ID %u）\n", cfg.port, cfg.unit_id);
```

成功後 Server 在背景執行緒中監聽，主執行緒可繼續執行其他工作。

---

## 8. 連線事件通知（on_link，可選）

若需要追蹤 Master 的連線與斷線事件，實作 `mb_tcp_link_fn` 並填入 `cfg.on_link`：

**函式簽章：**

```c
typedef void (*mb_tcp_link_fn)(
    void *userdata,  // 對應 cfg.link_userdata
    int   fd,        // Client Socket fd（斷線時為已關閉的 fd）
    int   connected  // 1 = 已連線，0 = 已斷線
);
```

**追蹤連線數量的範例：**

```c
typedef struct {
    volatile int active_connections;
} link_ctx_t;

static void on_link_change(void *userdata, int fd, int connected)
{
    link_ctx_t *ctx = (link_ctx_t *)userdata;
    if (connected) {
        ctx->active_connections++;
        printf("Master 已連線 (fd=%d)，目前連線數: %d\n",
               fd, ctx->active_connections);
    } else {
        if (ctx->active_connections > 0) ctx->active_connections--;
        printf("Master 已斷線 (fd=%d)，目前連線數: %d\n",
               fd, ctx->active_connections);
    }
}

// 在設定中填入
link_ctx_t link_ctx = {.active_connections = 0};
cfg.on_link       = on_link_change;
cfg.link_userdata = &link_ctx;
```

`link_ctx.active_connections` 可從任意執行緒讀取，用來判斷目前是否有 Master 在線，進而決定設備是否進入安全狀態。

---

## 9. 停止 Server

```c
mb_tcp_server_stop(&server);
```

阻塞呼叫，會關閉所有 Client 連線並等待監聽執行緒結束後才回傳。

---

## 10. 日誌（可選）

實作 `mb_tcp_logv_fn` 並填入 `cfg.logv`：

```c
typedef struct {
    const char        *node_name;
    mb_tcp_log_level_t min_level;
} my_log_ctx_t;

static void my_logv(void *userdata, mb_tcp_log_level_t level,
                    const char *fmt, va_list ap)
{
    my_log_ctx_t *ctx = (my_log_ctx_t *)userdata;
    if (level < ctx->min_level) return;

    static const char * const tags[] = {"DEBUG", "INFO ", "WARN ", "ERROR"};
    fprintf(stderr, "[%s][%s] ", ctx->node_name, tags[level]);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
}

my_log_ctx_t log_ctx = {
    .node_name = "MODBUS-SRV",
    .min_level = MB_TCP_LOG_INFO,
};

cfg.logv         = my_logv;
cfg.log_userdata = &log_ctx;
```

**日誌等級：**

| 等級 | 說明 |
|------|------|
| `MB_TCP_LOG_DEBUG` | 所有訊息（連線細節、封包解析等） |
| `MB_TCP_LOG_INFO` | 一般資訊以上 |
| `MB_TCP_LOG_WARN` | 警告與錯誤 |
| `MB_TCP_LOG_ERROR` | 僅錯誤 |

---

## 11. 完整範例

```c
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include "modbus_tcp_server.h"

#define REG_TEMPERATURE  0x00
#define REG_SETPOINT     0x0A
#define REG_CONTROL      0x0B
#define REG_BANK_SIZE    32

static uint16_t        reg_bank[REG_BANK_SIZE];
static pthread_mutex_t reg_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int    g_running = 1;

static void on_signal(int s) { (void)s; g_running = 0; }

static int on_read(uint8_t function_code, uint16_t addr, uint16_t qty,
                   uint16_t *out, void *userdata)
{
    (void)userdata;
    (void)function_code;

    if ((uint32_t)addr + qty > REG_BANK_SIZE)
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;

    pthread_mutex_lock(&reg_lock);
    reg_bank[REG_TEMPERATURE] = 253;
    for (uint16_t i = 0; i < qty; i++) out[i] = reg_bank[addr + i];
    pthread_mutex_unlock(&reg_lock);
    return 0;
}

static int on_write(uint16_t addr, uint16_t qty,
                    const uint16_t *data, void *userdata)
{
    (void)userdata;

    if ((uint32_t)addr + qty > REG_BANK_SIZE)
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;

    pthread_mutex_lock(&reg_lock);
    for (uint16_t i = 0; i < qty; i++) reg_bank[addr + i] = data[i];
    pthread_mutex_unlock(&reg_lock);

    for (uint16_t i = 0; i < qty; i++) {
        if ((addr + i) == REG_SETPOINT)
            printf("Setpoint → %u\n", data[i]);
    }
    return 0;
}

int main(void)
{
    signal(SIGINT, on_signal);
    memset(reg_bank, 0, sizeof(reg_bank));

    mb_tcp_server_ctx_t    server = {0};
    mb_tcp_server_config_t cfg;

    mb_tcp_server_config_init(&cfg);  // 填入所有可選欄位的安全預設值
    cfg.port     = 502;
    cfg.unit_id  = 1;
    cfg.on_read  = on_read;
    cfg.on_write = on_write;

    if (mb_tcp_server_start(&server, &cfg) != 0) {
        fprintf(stderr, "Server 啟動失敗\n");
        return 1;
    }
    printf("Modbus TCP Server 運行中（Port 502，Unit ID 1）\n");

    while (g_running) sleep(1);

    mb_tcp_server_stop(&server);
    printf("Server 已停止\n");
    return 0;
}
```

---

## 12. 注意事項

**執行緒安全**
`on_read` 與 `on_write` 可能被多個 Master 的服務執行緒同時呼叫。若回呼中存取共用狀態（如 `reg_bank`），**必須自行加鎖**保護。

**地址範圍驗證**
回呼中務必驗證 `(uint32_t)addr + qty > REG_BANK_SIZE`，防止緩衝區溢位。加法必須以 `uint32_t` 進行，避免 `uint16_t` 相加時整數溢位。

**回呼的回傳值應精確**
盡量回傳正確的 Exception Code 而非一律回傳非零值，這樣 Master 端才能得到有意義的診斷資訊。例如地址超出範圍應回傳 `MODBUS_EX_ILLEGAL_DATA_ADDRESS`，而不是 `-1`。

**地址規範**
所有暫存器地址為 **0-based**。Modbus 地址 `40001` 對應 offset `0`，以此類推。

**Port 權限**
在 Linux 上監聽 Port 502（< 1024）需要 `root` 權限或 `CAP_NET_BIND_SERVICE` 能力。開發測試時可改用 5502 等高位 Port。

**Context 生命週期**
`mb_tcp_server_start()` 呼叫後，`cfg` 中的指標欄位（`on_read`、`on_write`、`logv` 等）必須在 `mb_tcp_server_stop()` 回傳前持續有效。
