# Modbus TCP Client 使用說明

`modbus_tcp_client.h` 提供一套阻塞式、執行緒安全的 Modbus TCP Client（Master）API，讓應用層可以直接讀寫遠端 Modbus Server 的暫存器，無需處理底層 Socket 或 Modbus 封包格式。

---

## 目錄

1. [快速開始](#1-快速開始)
2. [引入標頭檔](#2-引入標頭檔)
3. [定義暫存器地圖](#3-定義暫存器地圖)
4. [設定與連線](#4-設定與連線)
5. [讀取暫存器](#5-讀取暫存器)
6. [寫入暫存器](#6-寫入暫存器)
7. [回傳值與錯誤處理](#7-回傳值與錯誤處理)
8. [斷線](#8-斷線)
9. [日誌（可選）](#9-日誌可選)
10. [完整範例](#10-完整範例)
11. [注意事項](#11-注意事項)

---

## 1. 快速開始

```c
mb_tcp_client_ctx_t client = {0};

mb_tcp_client_config_t cfg = {
    .remote_host         = "192.168.1.90",
    .port                = 502,
    .unit_id             = 1,
    .connect_timeout_sec = 5,
    .response_timeout_ms = 2000,
    .logv                = NULL,
    .log_userdata        = NULL,
};

mb_tcp_client_connect(&client, &cfg);

uint16_t value = 0;
mb_tcp_client_read_holding_registers(&client, 0x00, 1, &value);

mb_tcp_client_disconnect(&client);
```

---

## 2. 引入標頭檔

```c
#include "modbus_tcp_client.h"
```

---

## 3. 定義暫存器地圖

建議以具名常數定義暫存器地址，方便維護。所有地址均為 **0-based**（Modbus 地址 40001 = offset 0）：

```c
// R   = 唯讀   R/W = 可讀可寫

#define REG_TEMPERATURE   0x00   // R   – 溫度 × 10（253 = 25.3 °C）
#define REG_HUMIDITY      0x01   // R   – 相對濕度 %
#define REG_STATUS        0x02   // R   – 狀態 bitmask
#define REG_UPTIME_LOW    0x03   // R   – 運行秒數（低 16 bit）
#define REG_UPTIME_HIGH   0x04   // R   – 運行秒數（高 16 bit）
#define REG_SETPOINT      0x0A   // R/W – 目標設定值
#define REG_CONTROL       0x0B   // R/W – 控制旗標
#define REG_DEVICE_ID     0x14   // R   – 裝置識別碼
```

---

## 4. 設定與連線

### 4.1 宣告 Context

```c
mb_tcp_client_ctx_t client = {0};  // 必須以 {0} 或 memset 零初始化
```

### 4.2 填寫設定

```c
mb_tcp_client_config_t cfg = {
    .remote_host         = "192.168.1.90", // Server IPv4 位址（必填）
    .port                = 502,            // Modbus 標準 Port（生產環境請使用 502）
    .unit_id             = 1,              // Modbus Unit / Slave ID（必填）
    .bind_iface          = NULL,           // 綁定網卡（如 "eth0"），NULL 表示由系統決定
    .connect_timeout_sec = 5,             // TCP 連線逾時秒數（0 = 預設 5 秒）
    .response_timeout_ms = 2000,          // 每次請求逾時毫秒（0 = 預設 1000 ms）
    .logv                = NULL,           // Log callback，NULL = 靜默模式
    .log_userdata        = NULL,           // 傳入 logv 的自訂指標
};
```

| 欄位 | 說明 | 填 0 / NULL 時的預設行為 |
|------|------|------------------------|
| `remote_host` | Server IPv4 字串 | — 必填 |
| `port` | TCP Port | — 必填 |
| `unit_id` | Modbus Unit ID | — 必填 |
| `bind_iface` | 綁定網路介面 | 系統依路由表決定 |
| `connect_timeout_sec` | TCP 連線逾時 | 5 秒 |
| `response_timeout_ms` | 每次請求逾時 | 1000 ms |
| `logv` | Log 回呼函式 | 靜默，不輸出任何訊息 |
| `log_userdata` | Log 回呼的自訂指標 | `NULL` |

### 4.3 建立連線

```c
if (mb_tcp_client_connect(&client, &cfg) != 0) {
    fprintf(stderr, "連線失敗\n");
    return -1;
}
```

成功回傳 `0`，失敗回傳 `-1`。

---

## 5. 讀取暫存器

### FC03 – Read Holding Registers

```c
int mb_tcp_client_read_holding_registers(
    mb_tcp_client_ctx_t *ctx,
    uint16_t addr,   // 起始暫存器地址（0-based）
    uint16_t qty,    // 讀取數量（1 ~ 125）
    uint16_t *out    // 呼叫端提供的緩衝區，接收 qty 個值（Host byte order）
);
```

**讀取單一暫存器：**

```c
uint16_t temperature = 0;
int rc = mb_tcp_client_read_holding_registers(&client, REG_TEMPERATURE, 1, &temperature);
if (rc == MB_TCP_CLIENT_OK) {
    printf("Temperature: %.1f °C\n", temperature / 10.0f);
}
```

**讀取連續暫存器區塊（效率較佳）：**

一次請求讀取多個相鄰暫存器，比分次呼叫更有效率。緩衝區索引對應起始地址的偏移量：

```c
uint16_t block[5] = {0};
// 讀取 REG_TEMPERATURE(0x00) 起連續 5 個暫存器
int rc = mb_tcp_client_read_holding_registers(&client, REG_TEMPERATURE, 5, block);
if (rc == MB_TCP_CLIENT_OK) {
    float    temp   = block[0] / 10.0f;   // REG_TEMPERATURE
    uint16_t hum    = block[1];            // REG_HUMIDITY
    uint16_t status = block[2];            // REG_STATUS
    // 32-bit uptime：兩個 16-bit 暫存器合併
    uint32_t uptime = ((uint32_t)block[4] << 16) | block[3];
    printf("Temp: %.1f °C, Humidity: %u %%, Uptime: %u s\n", temp, hum, uptime);
}
```

> 單次最多可讀取 **125** 個暫存器（`MODBUS_MAX_READ_REGISTERS`）。

---

## 6. 寫入暫存器

### FC06 – Write Single Register

寫入單一暫存器。若設備規格要求或偏好使用 FC06，請選用此函式：

```c
int mb_tcp_client_write_single_register(
    mb_tcp_client_ctx_t *ctx,
    uint16_t addr,   // 暫存器地址（0-based）
    uint16_t value   // 要寫入的值（Host byte order）
);
```

```c
int rc = mb_tcp_client_write_single_register(&client, REG_SETPOINT, 300);
if (rc == MB_TCP_CLIENT_OK) {
    printf("Setpoint 設定為 300\n");
}
```

### FC16 – Write Multiple Registers

一次寫入一或多個連續暫存器。即使只寫 1 個，也可以使用此函式（Modbus 規範明確允許）：

```c
int mb_tcp_client_write_multiple_registers(
    mb_tcp_client_ctx_t *ctx,
    uint16_t addr,         // 起始暫存器地址（0-based）
    uint16_t qty,          // 寫入數量（1 ~ 123）
    const uint16_t *data   // 要寫入的值陣列（Host byte order）
);
```

```c
// 同時寫入 REG_SETPOINT 與 REG_CONTROL（相鄰暫存器）
uint16_t values[2] = {300, 0x0001};
int rc = mb_tcp_client_write_multiple_registers(&client, REG_SETPOINT, 2, values);
if (rc == MB_TCP_CLIENT_OK) {
    printf("Setpoint + Control 寫入成功\n");
}
```

> 單次最多可寫入 **123** 個暫存器（`MODBUS_MAX_WRITE_REGISTERS`）。

### FC06 vs FC16 選擇建議

| 情境 | 建議 |
|------|------|
| 設備規格指定使用 FC06 | 使用 `write_single_register` |
| 需要同時寫入多個相鄰暫存器 | 使用 `write_multiple_registers`（效率較佳） |
| 只寫 1 個但偏好 FC16 | 使用 `write_multiple_registers`，qty=1（合法） |

---

## 7. 回傳值與錯誤處理

所有讀寫函式的回傳值遵循同一套規則：

| 回傳值 | 意義 |
|--------|------|
| `0`（`MB_TCP_CLIENT_OK`） | 成功 |
| `> 0`（正整數） | Modbus Exception Code，Server 端拒絕請求 |
| `< 0`（負整數） | 傳輸或封包層錯誤 |

**負值錯誤碼說明：**

| 常數 | 值 | 說明 |
|------|----|------|
| `MB_TCP_CLIENT_ERR_ARG` | -1 | 傳入參數無效（如 qty=0 或指標為 NULL） |
| `MB_TCP_CLIENT_ERR_NOT_CONNECTED` | -2 | 尚未連線，請先呼叫 `connect` |
| `MB_TCP_CLIENT_ERR_TRANSPORT` | -3 | Socket 傳送或接收錯誤 |
| `MB_TCP_CLIENT_ERR_TIMEOUT` | -4 | 在 `response_timeout_ms` 內未收到回應 |
| `MB_TCP_CLIENT_ERR_FRAME` | -5 | 收到格式錯誤或非預期的回應封包 |
| `MB_TCP_CLIENT_ERR_TID` | -6 | Transaction ID 不符合 |

**Modbus Exception Code（正值）說明：**

| 值 | 常數 | 說明 |
|----|------|------|
| 1 | `MODBUS_EX_ILLEGAL_FUNCTION` | 不支援的功能碼 |
| 2 | `MODBUS_EX_ILLEGAL_DATA_ADDRESS` | 地址超出 Server 允許範圍 |
| 3 | `MODBUS_EX_ILLEGAL_DATA_VALUE` | 數值不合法 |
| 4 | `MODBUS_EX_SERVER_DEVICE_FAILURE` | Server 內部錯誤 |

**建議的錯誤處理模式：**

```c
static const char *err_to_str(int rc) {
    switch (rc) {
        case MB_TCP_CLIENT_OK:                return "OK";
        case MB_TCP_CLIENT_ERR_ARG:           return "ERR_ARG";
        case MB_TCP_CLIENT_ERR_NOT_CONNECTED: return "ERR_NOT_CONNECTED";
        case MB_TCP_CLIENT_ERR_TRANSPORT:     return "ERR_TRANSPORT";
        case MB_TCP_CLIENT_ERR_TIMEOUT:       return "ERR_TIMEOUT";
        case MB_TCP_CLIENT_ERR_FRAME:         return "ERR_FRAME";
        case MB_TCP_CLIENT_ERR_TID:           return "ERR_TID";
        default: return (rc > 0) ? "MODBUS_EXCEPTION" : "UNKNOWN";
    }
}

int rc = mb_tcp_client_read_holding_registers(&client, REG_SETPOINT, 1, &val);
if (rc != MB_TCP_CLIENT_OK) {
    fprintf(stderr, "讀取失敗: %s (rc=%d)\n", err_to_str(rc), rc);
}
```

**傳輸錯誤後的處理：**

`MB_TCP_CLIENT_ERR_TRANSPORT` 與 `MB_TCP_CLIENT_ERR_TIMEOUT` 代表 Socket 層面已出現問題。此時應先斷線再重新連線：

```c
int rc = mb_tcp_client_read_holding_registers(&client, addr, qty, buf);
if (rc == MB_TCP_CLIENT_ERR_TRANSPORT || rc == MB_TCP_CLIENT_ERR_TIMEOUT) {
    mb_tcp_client_disconnect(&client);
    // 視應用需求決定重連時機與重試策略
    mb_tcp_client_connect(&client, &cfg);
}
```

---

## 8. 斷線

```c
mb_tcp_client_disconnect(&client);
```

即使 `connect` 失敗也可以安全呼叫。此函式會關閉 Socket 並釋放 Mutex，為阻塞呼叫，回傳後資源即已全部釋放。

---

## 9. 日誌（可選）

若需要診斷訊息，實作 `mb_tcp_logv_fn` 並填入 `cfg.logv`：

```c
typedef struct {
    const char        *device_name;
    mb_tcp_log_level_t min_level;   // 低於此等級的訊息會被過濾
} my_log_ctx_t;

static void my_logv(void *userdata, mb_tcp_log_level_t level,
                    const char *fmt, va_list ap)
{
    my_log_ctx_t *ctx = (my_log_ctx_t *)userdata;
    if (level < ctx->min_level) return;

    static const char * const tags[] = {"DEBUG", "INFO ", "WARN ", "ERROR"};
    fprintf(stderr, "[%s][%s] ", ctx->device_name, tags[level]);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
}

// 在設定中填入
my_log_ctx_t log_ctx = {
    .device_name = "SENSOR-01",
    .min_level   = MB_TCP_LOG_INFO,
};

mb_tcp_client_config_t cfg = {
    // ...
    .logv          = my_logv,
    .log_userdata  = &log_ctx,
};
```

**日誌等級：**

| 等級 | 說明 |
|------|------|
| `MB_TCP_LOG_DEBUG` | 所有訊息（連線細節、TID 等） |
| `MB_TCP_LOG_INFO` | 一般資訊以上 |
| `MB_TCP_LOG_WARN` | 警告與錯誤 |
| `MB_TCP_LOG_ERROR` | 僅錯誤 |

不需要 Log 時，設定 `cfg.logv = NULL` 即完全靜默。

---

## 10. 完整範例

```c
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include "modbus_tcp_client.h"

#define SERVER_IP   "192.168.1.90"
#define SERVER_PORT 502
#define UNIT_ID     1

#define REG_TEMPERATURE 0x00
#define REG_SETPOINT    0x0A
#define REG_CONTROL     0x0B

static volatile int g_running = 1;
static void on_signal(int s) { (void)s; g_running = 0; }

static const char *err_to_str(int rc) {
    switch (rc) {
        case MB_TCP_CLIENT_OK:                return "OK";
        case MB_TCP_CLIENT_ERR_ARG:           return "ERR_ARG";
        case MB_TCP_CLIENT_ERR_NOT_CONNECTED: return "ERR_NOT_CONNECTED";
        case MB_TCP_CLIENT_ERR_TRANSPORT:     return "ERR_TRANSPORT";
        case MB_TCP_CLIENT_ERR_TIMEOUT:       return "ERR_TIMEOUT";
        case MB_TCP_CLIENT_ERR_FRAME:         return "ERR_FRAME";
        case MB_TCP_CLIENT_ERR_TID:           return "ERR_TID";
        default: return (rc > 0) ? "MODBUS_EXCEPTION" : "UNKNOWN";
    }
}

int main(void)
{
    signal(SIGINT, on_signal);

    mb_tcp_client_ctx_t    client = {0};
    mb_tcp_client_config_t cfg    = {
        .remote_host         = SERVER_IP,
        .port                = SERVER_PORT,
        .unit_id             = UNIT_ID,
        .connect_timeout_sec = 5,
        .response_timeout_ms = 2000,
        .logv                = NULL,
    };

    if (mb_tcp_client_connect(&client, &cfg) != 0) {
        fprintf(stderr, "連線失敗\n");
        return 1;
    }
    printf("已連線至 %s:%u\n", SERVER_IP, SERVER_PORT);

    // 啟動時寫入初始值（FC16：同時寫入 setpoint 與 control）
    uint16_t init[2] = {300, 0x0001};
    int rc = mb_tcp_client_write_multiple_registers(&client, REG_SETPOINT, 2, init);
    if (rc == MB_TCP_CLIENT_OK)
        printf("初始值寫入成功\n");
    else
        fprintf(stderr, "初始值寫入失敗: %s (rc=%d)\n", err_to_str(rc), rc);

    // 輪詢迴圈
    while (g_running) {
        uint16_t temp = 0;
        rc = mb_tcp_client_read_holding_registers(&client, REG_TEMPERATURE, 1, &temp);
        if (rc == MB_TCP_CLIENT_OK)
            printf("Temperature: %.1f °C\n", temp / 10.0f);
        else
            fprintf(stderr, "讀取失敗: %s (rc=%d)\n", err_to_str(rc), rc);
        sleep(2);
    }

    mb_tcp_client_disconnect(&client);
    printf("已斷線\n");
    return 0;
}
```

---

## 11. 注意事項

**執行緒安全**
`mb_tcp_client_ctx_t` 支援多執行緒共用。API 內部以 Mutex 序列化所有請求，同一時間只有一個請求在進行中。

**Context 生命週期**
`mb_tcp_client_connect()` 呼叫後，`cfg` 中的指標欄位（如 `remote_host`、`bind_iface`）必須在 `mb_tcp_client_disconnect()` 回傳前持續有效。

**支援的功能碼**
此 Client 僅支援：FC03（Read Holding Registers）、FC06（Write Single Register）、FC16（Write Multiple Registers）。

**地址規範**
所有暫存器地址為 **0-based**。Modbus 地址 `40001` 對應 offset `0`，`40010` 對應 offset `9`，以此類推。
