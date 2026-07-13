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
RTU 沒有 TCP Transaction ID，因此 `-6` 代表 `MB_RTU_CLIENT_ERR_UNIT_ID`。

---

## 支援的 Function Codes

| FC | 名稱 | Server 接收 | Client 發送 |
|---|---|---|---|
| 0x03 | Read Holding Registers | ✅ | ✅ |
| 0x04 | Read Input Registers | ✅ | — |
| 0x06 | Write Single Register | ✅ | ✅ |
| 0x10 | Write Multiple Registers | ✅ | ✅ |

未支援的 FC 會收到 `MODBUS_EX_ILLEGAL_FUNCTION` exception 回應。
