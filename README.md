# Modbus TCP / RTU Library

A lightweight, dependency-free Modbus TCP / RTU library for Linux, written in C11.

## 目錄結構

```
Modbus_protocol/
├── include/
│   ├── modbus_defines.h       # Modbus 常數、FC codes、exception codes
│   ├── modbus_tcp.h           # 底層 transport（socket / thread）— 通常不直接使用
│   ├── modbus_tcp_server.h    # Server API
│   ├── modbus_tcp_client.h    # Client API
│   ├── modbus_rtu.h           # 底層 RTU transport / CRC / serial helpers
│   ├── modbus_rtu_server.h    # RTU Server API
│   └── modbus_rtu_client.h    # RTU Client API
├── src/
│   ├── modbus_tcp.c           # 底層 transport 實作
│   ├── modbus_tcp_server.c    # Server API 實作
│   ├── modbus_tcp_client.c    # Client API 實作
│   ├── modbus_rtu.c           # RTU serial / CRC / framing 實作
│   ├── modbus_rtu_server.c    # RTU Server API 實作
│   └── modbus_rtu_client.c    # RTU Client API 實作
├── examples/
│   ├── tcp_server_example.c   # TCP Server 範例（改這個讓你的裝置被讀寫）
│   ├── tcp_client_example.c   # TCP Client 範例（改這個去讀寫遠端裝置）
│   ├── rtu_server_example.c   # RTU Server 範例
│   └── rtu_client_example.c   # RTU Client 範例
├── test/
│   ├── test_modbus_tcp.c      # TCP end-to-end 自動化驗證
│   └── test_modbus_rtu.c      # RTU framing / CRC 自動化驗證
└── Makefile
```

---

## 環境需求

| 項目 | 版本 |
|---|---|
| GCC | >= 7（需支援 C11） |
| GNU Make | >= 4 |
| pthreads | glibc 標準，無需額外安裝 |
| OS | Linux（使用 SO_KEEPALIVE、SO_BINDTODEVICE 等 Linux socket API） |

---

## 編譯

### 架構選擇

Makefile 預設編譯 **ARM**，可透過 `ARCH=` 參數切換：

| 指令 | 目標架構 | 使用的編譯器 | 輸出目錄 |
|---|---|---|---|
| `make` | ARM64（預設） | `aarch64-linux-gnu-gcc` | `build/arm/` |
| `make ARCH=arm` | ARM64 | `aarch64-linux-gnu-gcc` | `build/arm/` |
| `make ARCH=x86` | x86（本機） | `gcc` | `build/x86/` |

> **ARM64 cross-compiler 安裝（若尚未安裝）：**
> ```bash
> sudo apt install gcc-aarch64-linux-gnu
> ```

---

### 1. 編譯靜態函式庫

```bash
make               # ARM（預設）
make ARCH=x86      # x86
```

產出：
- `build/arm/libmodbus_protocol.a`
- `build/x86/libmodbus_protocol.a`

---

### 2. 編譯範例程式

```bash
make examples              # ARM（預設）
make examples ARCH=x86     # x86
```

產出（以 ARM 為例）：
- `build/arm/tcp_server_example`
- `build/arm/tcp_client_example`
- `build/arm/rtu_server_example`
- `build/arm/rtu_client_example`

---

### 3. 執行自動化驗證測試

測試程式需在目標機器上執行，**ARM binary 無法直接在 x86 host 上運行**：

```bash
make test ARCH=x86     # 在 x86 host 上直接執行（開發驗證用）
make test              # 只編譯 ARM binary，並提示需部署到板子上執行
```

測試項目：
- FC03 Read Holding Registers
- FC06 Write Single Register
- FC16 Write Multiple Registers
- Exception 處理（超出範圍、非法參數）
- logv callback 驗證
- 斷線後呼叫保護
- RTU CRC16、request parser、response builder 驗證
- RTU 設備存在探測（mb_rtu_client_probe_device）client 端參數驗證

---

### 4. 查看目前編譯設定

```bash
make info              # ARM
make info ARCH=x86     # x86
```

---

### 5. 清除編譯產出

```bash
make clean             # 清除目前 ARCH 的輸出（預設清 build/arm/）
make clean ARCH=x86    # 清除 x86 輸出
make clean-all         # 清除所有架構的輸出（整個 build/）
```

---

## 快速上機測試步驟

### Step 1：修改範例程式

**如果你的裝置是 TCP Slave（被讀寫）→ 修改 `examples/tcp_server_example.c`**

開啟檔案，找到以下四個 `(MODIFY HERE)` 區塊：

```
① NETWORK SETTINGS  — 設定 port 和 unit ID
② REGISTER MAP      — 定義你的暫存器名稱和位址
③ READ CALLBACK     — FC03/FC04 被呼叫時，回傳你的硬體數值
④ WRITE CALLBACK    — FC06/FC16 被呼叫時，把數值寫入你的硬體
```

**如果你的裝置是 TCP Master（主動讀寫）→ 修改 `examples/tcp_client_example.c`**

```
① NETWORK SETTINGS  — 設定目標 IP、port、unit ID、timeout
② REGISTER MAP      — 對應 server 的暫存器定義
③ POLL LOOP         — 每次輪詢讀哪些暫存器、如何解讀數值
④ WRITE EXAMPLE     — 啟動時要寫入的初始值
```

RTU 使用方式相同，改用：
- Slave：`examples/rtu_server_example.c`
- Master：`examples/rtu_client_example.c`

RTU 主要設定欄位：

```c
cfg.device    = "/dev/ttyUSB0";
cfg.baud_rate = 9600u;
cfg.data_bits = 8u;
cfg.stop_bits = 1u;
cfg.parity    = MB_RTU_PARITY_NONE;
cfg.unit_id   = 1u;
```

RTU Client / Server 時序欄位語意（不修改 lib 即可調整）：

| 設定值 | 意義 |
|--------|------|
| `0` | 使用 library 預設 |
| `MB_RTU_TIMING_DISABLED` | 明確關閉該等待 |
| 其他正值 | 使用該明確延遲 |

```c
/* Client */
cfg.response_timeout_ms    = 1000u;  // 等待回應逾時；0 = 1 s
cfg.inter_frame_delay_us   = 0u;     // request 前 T3.5；0 = 自動，DISABLED = 關
cfg.bus_handoff_silence_ms = 0u;     // 換 unit_id 靜默；0 = 30 ms，DISABLED = 關

/* Server（回覆前） */
cfg.inter_frame_delay_us   = 0u;     // response 前 T3.5；0 = 自動，DISABLED = 關
```

Client 的 `inter_frame_delay_us` 與 `bus_handoff_silence_ms` 從同一埠上一筆交易結束起算，取較大值。
Server 的 `inter_frame_delay_us` 從收到 request 最後一 byte 起算，回覆前補足剩餘靜默。
自動 T3.5 可用 `mb_rtu_t35_us()` 取得（> 19200 baud 固定 1750 us）。
同一個 `device` 的多個 client context 共用一個 fd 與 bus lock，序列埠參數須一致。

---

### Step 2：重新編譯

```bash
make examples
```

---

### Step 3：執行

開兩個終端機分別執行 server 和 client：

```bash
# 終端機 1
./build/arm/tcp_server_example

# 終端機 2
./build/arm/tcp_client_example
```

---

## 關於 Port 502

Modbus TCP 標準 port 為 **502**，但 Linux 上 port < 1024 需要 root 或特殊權限。

**開發 / 測試階段**（port > 1024，無需特殊設定）：

```c
// tcp_server_example.c
#define SERVER_PORT  15030u
```

**生產部署**（port 502，二選一）：

```bash
# 方法 A：以 root 執行
sudo ./build/arm/tcp_server_example

# 方法 B：授予 CAP_NET_BIND_SERVICE（推薦，不需完整 root）
sudo setcap cap_net_bind_service+ep ./build/arm/tcp_server_example
./build/arm/tcp_server_example
```

然後把程式碼裡的 port 改回：

```c
#define SERVER_PORT  502u
```

---

## Log 輸出

範例程式的 log 預設輸出到 **stderr**，格式：

```
[HH:MM:SS][SERVER][LEVEL] 訊息內容
[HH:MM:SS][CLIENT][LEVEL] 訊息內容
```

儲存到檔案：

```bash
./build/arm/tcp_server_example 2>> server.log
./build/arm/tcp_client_example 2>> client.log
```

同時顯示在終端機並儲存：

```bash
./build/arm/tcp_server_example 2>&1 | tee server.log
```

Log 等級（由低到高）：`DEBUG` → `INFO` → `WARN` → `ERROR`

---

## 將 Library 整合進你的專案

只需引用兩個標頭：

```c
#include "modbus_tcp_server.h"   // Server 用
#include "modbus_tcp_client.h"   // Client 用

#include "modbus_rtu_server.h"   // RTU Server 用
#include "modbus_rtu_client.h"   // RTU Client 用
```

編譯時連結靜態函式庫和 pthread：

```bash
gcc your_app.c -Ipath/to/Modbus_protocol/include \
    -Lpath/to/Modbus_protocol/build/x86 \
    -lmodbus_protocol -lpthread \
    -o your_app
```

---

## API 快速參考

### Server

```c
// TCP 啟動（會建立 listener thread）
int mb_tcp_server_start(mb_tcp_server_ctx_t *ctx, const mb_tcp_server_config_t *cfg);

// TCP 停止（blocking，等待 thread 結束）
void mb_tcp_server_stop(mb_tcp_server_ctx_t *ctx);

// RTU 啟動 / 停止（API 習慣與 TCP 相同）
int mb_rtu_server_start(mb_rtu_server_ctx_t *ctx, const mb_rtu_server_config_t *cfg);
void mb_rtu_server_stop(mb_rtu_server_ctx_t *ctx);
```

### Client

```c
// TCP 連線
int mb_tcp_client_connect(mb_tcp_client_ctx_t *ctx, const mb_tcp_client_config_t *cfg);

// TCP 斷線
void mb_tcp_client_disconnect(mb_tcp_client_ctx_t *ctx);

// 讀取 holding registers（FC03）
int mb_tcp_client_read_holding_registers(mb_tcp_client_ctx_t *ctx,
                                         uint16_t addr, uint16_t qty, uint16_t *out);

// 寫入 single / multiple registers
int mb_tcp_client_write_single_register(mb_tcp_client_ctx_t *ctx,
                                        uint16_t addr, uint16_t value);
int mb_tcp_client_write_multiple_registers(mb_tcp_client_ctx_t *ctx,
                                           uint16_t addr, uint16_t qty,
                                           const uint16_t *data);

// FC04 input registers
int mb_tcp_client_read_input_registers(mb_tcp_client_ctx_t *ctx,
                                       uint16_t addr, uint16_t qty, uint16_t *out);

// Coils / discrete inputs（FC01 / FC02 / FC05 / FC15）
// bit 緩衝區一個 byte 代表一個 bit：讀出為 0/1，寫入時非 0 即 ON
int mb_tcp_client_read_coils(mb_tcp_client_ctx_t *ctx,
                             uint16_t addr, uint16_t qty, uint8_t *out);
int mb_tcp_client_read_discrete_inputs(mb_tcp_client_ctx_t *ctx,
                                       uint16_t addr, uint16_t qty, uint8_t *out);
int mb_tcp_client_write_single_coil(mb_tcp_client_ctx_t *ctx,
                                    uint16_t addr, uint8_t value);
int mb_tcp_client_write_multiple_coils(mb_tcp_client_ctx_t *ctx,
                                       uint16_t addr, uint16_t qty,
                                       const uint8_t *values);

// 不斷線切換目標 unit id（gateway 後多台裝置時使用）
int mb_tcp_client_set_unit_id(mb_tcp_client_ctx_t *ctx, uint8_t unit_id);

// RTU 對應 API：function prefix 改為 mb_rtu_client_*
int mb_rtu_client_connect(mb_rtu_client_ctx_t *ctx, const mb_rtu_client_config_t *cfg);
void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx);
int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
                                         uint16_t addr, uint16_t qty, uint16_t *out);
int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
                                        uint16_t addr, uint16_t value);
int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
                                           uint16_t addr, uint16_t qty,
                                           const uint16_t *data);

// RTU 專用：探測 slave 是否真的存在並回應（connect() 只保證序列埠節點打開，
// RTU 無 handshake，故無法單靠 connect() 判斷對端是否在線）
int mb_rtu_client_probe_device(mb_rtu_client_ctx_t *ctx,
                               uint16_t addr, uint16_t qty);
```

### Client 回傳值

| 值 | 意義 |
|---|---|
| `0` (`MB_TCP_CLIENT_OK`) | 成功 |
| `> 0` | Modbus exception code（`MODBUS_EX_*`） |
| `MB_TCP_CLIENT_ERR_ARG` (-1) | 非法參數（在 client 端拒絕，未送出任何封包） |
| `MB_TCP_CLIENT_ERR_NOT_CONNECTED` (-2) | 尚未連線 |
| `MB_TCP_CLIENT_ERR_TRANSPORT` (-3) | Socket 傳輸錯誤 |
| `MB_TCP_CLIENT_ERR_TIMEOUT` (-4) | 等待回應逾時 |
| `MB_TCP_CLIENT_ERR_FRAME` (-5) | 收到格式錯誤的回應 |
| `MB_TCP_CLIENT_ERR_TID` (-6) | Transaction ID 不匹配 |

RTU client 使用同樣回傳規則，錯誤碼 prefix 為 `MB_RTU_CLIENT_ERR_*`；
RTU 沒有 TCP Transaction ID，因此 `-6` 代表 `MB_RTU_CLIENT_ERR_UNIT_ID`；
`-7` 為 `MB_RTU_CLIENT_ERR_CRC`（回應 CRC 錯誤）。

TCP client 逾時（完全沒收到回應）時連線保留，該請求的遲到回應會在下一次呼叫時依 TID 丟棄；
若串流已無法信任（send / recv 失敗、MBAP 長度錯誤、frame 收到一半逾時），client 會關閉 socket，
之後的呼叫回傳 `MB_TCP_CLIENT_ERR_NOT_CONNECTED`，需 `disconnect()` 後再 `connect()`。

`MB_*_CLIENT_ERR_FRAME` 也涵蓋：TCP 回應 Protocol ID 非 0、FC06 / FC16 寫入回應的
address / value / quantity 與請求不一致（TCP 與 RTU 皆檢查）。

### 生命週期規則

- ctx 第一次使用前須清零（`= {0}`）。
- `*_start()` / `*_connect()` 對已啟動的 ctx 會回傳 `-1`，不會覆蓋執行中的狀態。
- `*_stop()` / `*_disconnect()` 可重複呼叫，對未啟動或啟動失敗的 ctx 為 no-op。
- stop / disconnect 後可直接再次 start / connect，不需重新 memset。

### RTU 行為說明

- **共用匯流排**：同一個序列裝置（以 `realpath()` 正規化，`/dev/serial/by-id/...`
  symlink 視為同一個）的多個 client ctx 共用同一個 fd 與 bus lock。可開啟的不同裝置
  數量不設上限；最後一個 ctx 斷線後該埠即關閉。共用同一埠的 ctx 序列參數與
  `rs485_mode` 須一致，否則 connect 失敗。
- **Broadcast（unit id 0）**：只允許寫入類 FC（05 / 06 / 15 / 16），送出後不等待回應即回傳
  成功；讀取類 FC 於 client 端回傳 `MB_RTU_CLIENT_ERR_ARG`。Server 收到 broadcast 寫入會執行但不回覆。
- **Unit id 不符**：回應的 slave address 與請求不同時回傳 `MB_RTU_CLIENT_ERR_UNIT_ID`。
- **回應逾時**：`response_timeout_ms` 是整個回應的總期限（從送出請求後開始算），
  另外自動加上一個最大 RTU frame 在目前 baud 下的傳輸時間，因此慢速 baud 的大量讀取
  不會被誤判逾時，而逐 byte 拖延的 slave 也無法無限期佔住 bus。
- **Server 重新同步**：RTU server 會聽到同一條 RS-485 上其他 slave 的回應；
  CRC 錯誤或格式不符的資料會逐 byte 滑動跳過，直到找到有效的請求邊界。

---

## 支援的 Function Codes

| FC | 名稱 | Server 接收 | Client 發送 |
|---|---|---|---|
| 0x01 | Read Coils | ✅ `on_read_bits` | ✅ |
| 0x02 | Read Discrete Inputs | ✅ `on_read_bits` | ✅ |
| 0x03 | Read Holding Registers | ✅ `on_read` | ✅ |
| 0x04 | Read Input Registers | ✅ `on_read` | ✅ |
| 0x05 | Write Single Coil | ✅ `on_write_bits`（qty = 1） | ✅ |
| 0x06 | Write Single Register | ✅ `on_write`（qty = 1） | ✅ |
| 0x0F | Write Multiple Coils | ✅ `on_write_bits` | ✅ |
| 0x10 | Write Multiple Registers | ✅ `on_write` | ✅ |

TCP 與 RTU 支援相同的 FC。未支援的 FC，或對應 callback 為 `NULL` 的 FC，
會收到 `MODBUS_EX_ILLEGAL_FUNCTION` exception 回應。

Server callback 回傳值：`0` 成功；`MODBUS_EX_*`（01–06、08、0A、0B）原樣回傳為 exception；
其他非零值一律回傳 `MODBUS_EX_SERVER_DEVICE_FAILURE`。數量超出規格
（bit 讀 2000 / 寫 1968、register 讀 125 / 寫 123）、byte count 不符、FC05 值非
`0xFF00` / `0x0000` 時，server 直接回 `MODBUS_EX_ILLEGAL_DATA_VALUE`，不會呼叫 callback。

---

## 可配置項目（不修改 lib）

### TCP server

| 欄位 | `0` 的意義 | 說明 |
|---|---|---|
| `max_clients` | `MB_TCP_DEFAULT_MAX_CLIENTS`（10） | 同時連線數，執行期配置，無編譯期上限（受 `FD_SETSIZE` 限制） |
| `keepalive_idle_sec` | 5 s | 設為 `MB_TCP_KEEPALIVE_DISABLED` 完全關閉 keepalive |
| `keepalive_interval_sec` | 5 s | keepalive 探測間隔 |
| `keepalive_count` | 1 | 探測失敗幾次判定斷線 |
| `idle_poll_ms` | 5000 ms | 無事件時 server thread 的 select 週期 |

### RTU RS-485 方向控制（client 與 server 欄位相同）

| 欄位 | 說明 |
|---|---|
| `rs485_mode` | `MB_RTU_RS485_NONE`（預設）/ `RTS_ON_SEND` / `RTS_AFTER_SEND`；透過 Linux `TIOCSRS485` 由 kernel 控制 DE/RE。驅動不支援時 connect / start 失敗 |
| `on_tx_enable` / `tx_userdata` | 送出 frame 前呼叫 `(ud, fd, 1)`，`tcdrain()` 完成後呼叫 `(ud, fd, 0)`；用於 GPIO 控制收發器 |
| `local_echo` | 收發器會回送自己 TX 時設 `1`；送出後讀掉等長的 echo（client 會比對內容，不符回傳 `ERR_TRANSPORT`） |
