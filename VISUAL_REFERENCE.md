# Visual Reference: Code Structure & Data Flow

## State Machine Flow Diagrams

### External Flash State Machine (extFlashFunc)

```
                    [INIT]
                      |
                      v
    +--> [ERASE] --> [ERASE_WAIT] --> [ERASE_END] --+
    |                                                 |
    |    +--> [READ] --> [READ_END] --+               |
    |    |                             |              |
[READY] <--+------> [WRITE] --> [WRITE_WAIT] --> [WRITE_END] --+
    |    |                                                      |
    |    +- [ERROR]                                             |
    |                                                           |
    +-------------------------------------------------------<--+

States:
  [READY]       - Idle, waiting for command
  [ERASE]       - Enable write, start erase operation
  [ERASE_WAIT]  - Wait for erase command completion
  [ERASE_END]   - Poll for device ready
  [READ]        - Initiate read operation
  [READ_END]    - Wait for data reception
  [WRITE]       - Enable write, start page program
  [WRITE_WAIT]  - Wait for write command completion
  [WRITE_END]   - Poll for device ready
  [ERROR]       - Error occurred, needs recovery

Triggers:
  - extFlashCmd(size, addr, cmd) -> Transition from READY
  - Callback flags (Cmd, Rx, Tx, StatusMatch) -> State progression
  - extFlashFunc() polling -> Transition to next state
```

### Image Transfer State Machine (imagerFunc)

```
         [INIT]
           |
           v
[READY] <--+
  |
  +-- New Packet? --> [XFER_START]
  |                      |
  |                      v
  +-- Type? --> [XFER]   (loop for each page)
       |          |
       |          +-- Terminator? --> [XFER_FINISH]
       |                                  |
       |                                  v
       |                          [TBL_A_UPD]
       |                          / | | | \
       |                    [R][U][E][W]
       |                          |
       |                          v
       +-- No Image? --> [TBL_B_UPD]
            |             / | | | \
            |        [R][U][E][W]
            |             |
            +----------<--+
                |
                v
            [INIT] --> back to READY
            
            [ERROR] - Reset on failure

Legend:
  [x/y]     - Fork based on condition
  [R][U][E][W] - Sub-states: Read, Update, Erase, Write
```

---

## Data Structure Relationships

```
┌─────────────────────────────────────────────────────────┐
│                   Global State                          │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │ extFlash mtFlash (308 bytes)                     │  │
│  ├──────────────────────────────────────────────────┤  │
│  │ Flags:                                           │  │
│  │  - exFlashCmdCpltFlg         (from HAL)         │  │
│  │  - exFlashRxCpltFlg          (from HAL)         │  │
│  │  - exFlashTxCpltFlg          (from HAL)         │  │
│  │  - exFlashStatusMatchFlg     (from HAL)         │  │
│  │                                                  │  │
│  │ State:                                           │  │
│  │  - exFlashStage (eFlSt enum)                    │  │
│  │  - exFlashCmplt (last completed command)        │  │
│  │                                                  │  │
│  │ Operation:                                       │  │
│  │  - exFlashAddress    (flash address)            │  │
│  │  - exFlashDataSize   (bytes to transfer)        │  │
│  │  - exFlashErsSize    (erase block size)         │  │
│  │                                                  │  │
│  │ Buffers:                                         │  │
│  │  - exFlashTxData[256]  (write buffer)           │  │
│  │  - exFlashRxData[256]  (read buffer)            │  │
│  │  - exFlashID[3]        (device ID)              │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │ imgData imageDat (image state)                   │  │
│  ├──────────────────────────────────────────────────┤  │
│  │ Transfer Control:                                │  │
│  │  - imgNewPcktFl      (packet ready flag)        │  │
│  │  - imgStateFlag      (current state)            │  │
│  │  - updTblStg         (table update sub-state)   │  │
│  │                                                  │  │
│  │ Image Metadata:                                  │  │
│  │  - imgType           (firmware type)            │  │
│  │  - imgTypeTransfr    (transfer type)            │  │
│  │  - imgAddr           (flash address)            │  │
│  │  - imgSz             (cumulative size)          │  │
│  │  - imgCRC_IN/OUT     (CRC state)                │  │
│  │                                                  │  │
│  │ Data Buffers:                                    │  │
│  │  - imgLineBuff[256]  (working buffer)           │  │
│  │  - imgTableA[]       (metadata table A)         │  │
│  │  - imgTableB[]       (metadata table B)         │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │ QSPI Driver (HAL)                                │  │
│  ├──────────────────────────────────────────────────┤  │
│  │ extern QSPI_HandleTypeDef hqspi                 │  │
│  │ extern CRC_HandleTypeDef hcrc                   │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

---

## Call Flow: Image Transfer Complete

```
User/Host
    |
    v
[Image Data Packet Received]
    |
    v
imgParsePayld(debugParse)
    |
    +-> imgPayloadCheck(debugParse)
    |   - Extract address/data from buffer
    |   - Calculate checksum (XOR-based)
    |   - Validate against received checksum
    |   - Return 1=valid, 0=error
    |
    v
extFlashWriteGui(&imageDat)
    |
    +-> Validate image pointer
    +-> Copy imgLineBuff -> extFlash.exFlashTxData
    +-> Set extFlash.exFlashAddress
    +-> extFlashCmd(256, addr, exFlWrite)
    |       |
    |       +-> Set mtFlash.exFlashStage = exFlashWrite
    |
    v
imagerFunc() [Main Loop]
    |
    +-> extFlashFunc()
    |   |
    |   +-> State: exFlashWrite
    |   |   - Enable write
    |   |   - Transition to exFlashWriteWait
    |   |
    |   +-> State: exFlashWriteWait
    |   |   - Wait for TX complete flag
    |   |   - Start automatic polling
    |   |   - Transition to exFlashWriteEnd
    |   |
    |   +-> State: exFlashWriteEnd
    |   |   - Wait for status match
    |   |   - Transition to exFlashReady
    |   |
    |   +-> (Callbacks fired by hardware)
    |       - HAL_QSPI_TxCpltCallback() -> exFlashTxCpltFlg++
    |       - HAL_QSPI_StatusMatchCallback() -> exFlashStatusMatchFlg++
    |
    v
[Next packet or terminator]
    |
    v
All pages transferred?
    |
    +-- No --> Loop (get next page)
    |
    +-- Yes --> Terminator received
        |
        v
    imgVerify(imageDat)
        |
        +-> Check type matches
        +-> Check size matches accumulated size
        +-> Check CRC matches final CRC
        |
        v
    [Update Tables A & B]
    [Write metadata to flash]
    [Return to READY]
```

---

## Memory Layout Diagram

```
External Flash (512MB) Address Map:

0x00000000  ┌────────────────────────────────┐
            │  IMG_MNG_GOLD (256KB)          │  Golden Reference Images
0x00040000  ├────────────────────────────────┤
            │  IMG_QV_GOLD (256KB)           │
0x00080000  ├────────────────────────────────┤
            │  IMG_BAT_GOLD (256KB)          │
0x000C0000  ├────────────────────────────────┤
            │  IMG_OH_GOLD (256KB)           │
0x00100000  ├────────────────────────────────┤
            │  IMG_SP_GOLD (256KB)           │
0x00140000  ├────────────────────────────────┤
            │  IMG_BOOT_MNG_GOLD (256KB)     │
            │  IMG_BOOT_QV_GOLD              │
            │  IMG_BOOT_BAT_GOLD             │
            │  IMG_BOOT_OH_GOLD              │
0x00280000  ├────────────────────────────────┤
            │  IMG_MNG_RLS (256KB)           │  Release/Active Images
0x002C0000  ├────────────────────────────────┤  (User Updatable)
            │  IMG_QV_RLS (256KB)            │
0x00300000  ├────────────────────────────────┤
            │  IMG_BAT_RLS (256KB)           │
0x00340000  ├────────────────────────────────┤
            │  IMG_OH_RLS (256KB)            │
0x00380000  ├────────────────────────────────┤
            │  IMG_SP_RLS (256KB)            │
0x003C0000  ├────────────────────────────────┤
            │  IMG_BOOT_MNG_RLS              │
            │  IMG_BOOT_QV_RLS               │
            │  IMG_BOOT_BAT_RLS              │
            │  IMG_BOOT_OH_RLS               │
            │  IMG_BOOT_SP_RLS               │
0x004C0000  ├────────────────────────────────┤
            │  (Reserved)                    │
0x00500000  ├────────────────────────────────┤
            │  IMG_TABLE_A (4KB)             │  Metadata Tables
            │  ├─ Type | Size | CRC          │  (2 copies for redundancy)
            │  ├─ Type | Size | CRC          │
            │  └─ Type | Size | CRC          │
0x00501000  ├────────────────────────────────┤
            │  IMG_TABLE_B (4KB)             │
            │  ├─ Type | Size | CRC          │
            │  ├─ Type | Size | CRC          │
            │  └─ Type | Size | CRC          │
0x00502000  └────────────────────────────────┘

Each image: 256KB = 1024 pages of 256 bytes
```

---

## Error Handling Flow (Current vs. Recommended)

### Current Implementation ❌

```
Flash Operation
    |
    v
HAL Function Called
    |
    ├─ Success --> Continue
    |
    └─ Failure --> [IGNORED!] --> Continue (Undefined Behavior)


Callback Occurs
    |
    v
Flag Incremented (Could overflow)
    |
    v
State Machine Waits for Flag
    |
    └─ No Error Recovery
```

### Recommended Implementation ✅

```
Flash Operation
    |
    v
HAL Function Called
    |
    ├─ Success --> Continue
    |
    └─ Failure --> [CHECK] 
            |
            v
            Set Error State
            |
            v
            Log Error
            |
            v
            Attempt Recovery
            |
            ├─ Retry --> Reinitialize --> Retry
            |
            └─ Give Up --> Return Error to Caller


Callback Occurs
    |
    v
[CRITICAL SECTION] --> Flag Set to 1 (not incremented)
    |
    v
State Machine Processes Flag
    |
    v
[Error Check] 
    |
    ├─ Success --> Advance State
    |
    └─ Timeout --> Transition to Error State
```

---

## Function Call Hierarchy

```
main()
 |
 +-- imagerInit()
 |
 +-- extFlashInit()
 |   |
 |   +-- HAL_GPIO_WritePin()           [Error check missing]
 |   +-- MT25QL512ABB_ReadID()         [Error check missing]
 |   +-- MT25QL512ABB_ReadStatusReg()  [Error check missing]
 |   +-- MT25QL512ABB_EnterQPIMode()   [Error check missing]
 |   +-- MT25QL512ABB_Enter4BytesAddr()[Error check missing]
 |   +-- ... more HAL calls [Error check missing]
 |
 +-- Main Loop:
 |   |
 |   +-- imagerFunc()
 |   |   |
 |   |   +-- extFlashStatGet()
 |   |   |
 |   |   +-- extFlashCmdGet()
 |   |   |
 |   |   +-- extFlashReadGui()    [NULL check bug]
 |   |   |
 |   |   +-- extFlashWriteGui()   [NULL check bug]
 |   |   |
 |   |   +-- extFlashCmd()
 |   |   |   |
 |   |   |   +-- [State Machine Transition]
 |   |   |
 |   |   +-- imgTableUpdt()       [Array bounds bug]
 |   |   |   |
 |   |   |   +-- memcpy()
 |   |   |
 |   |   +-- imgVerify()
 |   |       |
 |   |       +-- [CRC Check]
 |   |       +-- [Size Check]
 |   |       +-- [Type Check]
 |   |
 |   +-- extFlashFunc()
 |   |   |
 |   |   +-- [State Machine Handler]
 |   |   |
 |   |   +-- MT25QL512ABB_*()     [All HAL calls]
 |   |
 |   +-- imgParsePayld()
 |       |
 |       +-- imgPayloadCheck()     [Complex circular buffer]
 |           |
 |           +-- readCircularBuffer() [Suggested refactor]
 |
 +-- Interrupt Handlers:
     |
     +-- HAL_QSPI_CmdCpltCallback()       [Overflow risk]
     +-- HAL_QSPI_RxCpltCallback()        [Overflow risk]
     +-- HAL_QSPI_TxCpltCallback()        [Overflow risk]
     +-- HAL_QSPI_StatusMatchCallback()   [Overflow risk]
```

---

## Data Flow: Writing an Image

```
Host System sends image over serial/network
                    |
                    v
            Receive Buffer (DMA)
                    |
                    v
            rxParseBuffDbg (circular)
                    |
                    v
            imgParsePayld()
                 /    \
              Valid    Error -> Return 0
               |
               v
            imgPayloadCheck()
            - Parse address (optional)
            - Extract 256-byte page
            - Calculate checksum
            - Validate
            |
            v
        imageDat.imgLineBuff[256]
            |
            v
        imagerFunc() [Main State Machine]
            |
            +-- Accumulate CRC over pages
            +-- Track total size
            +-- Copy page to flash buffer
            |
            v
        extFlashWriteGui()
            - Copy to mtFlash.exFlashTxData[256]
            - Set address
            - Call extFlashCmd(256, addr, exFlWrite)
            |
            v
        extFlashFunc() [Flash State Machine]
            |
            +-- Enable write
            +-- Issue PageProgram command
            +-- Wait for completion
            +-- Poll status until ready
            |
            v
        QSPI Hardware
            |
            +-- Transmit data buffer
            +-- Receive status register
            +-- Assert completion signals
            |
            v
        Interrupt Handlers
            |
            +-- HAL_QSPI_TxCpltCallback()
            +-- HAL_QSPI_StatusMatchCallback()
            |
            v
        Flag Variables (exFlashTxCpltFlg, exFlashStatusMatchFlg)
            |
            v
        State Machine Detects Completion
            |
            v
        Transition to exFlashReady
            |
            v
        Continue with next page or finalize
            |
            v
        All pages written?
            |
            +-- No --> Get next packet
            |
            +-- Yes --> Receive terminator with final CRC
                |
                v
            imgVerify()
            - Check type matches
            - Check accumulated size matches
            - Check final CRC matches
            |
            +-- Valid --> Update metadata tables
            |       |
            |       v
            |   Read Table A from flash
            |   Update entry
            |   Erase Table A block
            |   Write Table A back
            |   Read Table B from flash
            |   Update entry
            |   Erase Table B block
            |   Write Table B back
            |
            |       v
            |   Return to READY state
            |
            +-- Invalid --> Set ERROR state
                    |
                    v
                Discard transfer
```

---

## Buffer Management Overview

```
                    256 Bytes per Page
    
    exFlashRxData[256]   <------ Flash Read Data
    (read from device)           (Used by: extFlashReadGui)
           |
           v
    imgLineBuff[256]     <------ Working Buffer
    (image page data)           (Used by: imagerFunc, imgTableUpdt, imgVerify)
           |
           v
    exFlashTxData[256]   <------ Flash Write Buffer
    (write to device)           (Used by: extFlashWriteGui)
           |
           v
    QSPI Hardware


    Image Tables:
    imgTableA[]          <------ Metadata Table A
    (Multiple entries)           (Used by: imgTableUpdt)
           |
           v
    imgTableB[]          <------ Metadata Table B
    (Multiple entries)           (Used by: imgTableUpdt)

    
    Circular Receive Buffer:
    rxParseBuffDbg[N]    <------ Incoming Data
    (Circular with wraparound)   (Used by: imgPayloadCheck)
```

---

## Recommended Code Organization (Future)

```
drivers/
 ├── flash/
 │   ├── flash_driver.h        (Abstract interface)
 │   ├── flash_driver.c
 │   ├── qspi_flash.h          (QSPI implementation)
 │   └── qspi_flash.c
 │
 ├── qspi/
 │   ├── qspi_hal.h
 │   └── qspi_hal.c
 │
 └── crc/
     ├── crc_utils.h
     └── crc_utils.c

firmware/
 ├── imager.h                  (Interface)
 ├── imager.c                  (State machine)
 ├── image_protocol.h          (Protocol parsing)
 ├── image_protocol.c
 ├── image_validator.h         (CRC, verification)
 └── image_validator.c

tests/
 ├── test_flash_driver.c
 ├── test_imager.c
 ├── test_image_validator.c
 └── test_protocol.c
```

This structure provides better separation of concerns and testability.

