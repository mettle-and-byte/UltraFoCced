# Copycat Robot Arm Design Document

## 1. Overview
The "Copycat" system allows a 6DOF robot arm to record and replay movements without a central control board. Each joint is controlled by an independent "UltraFoCced" driver board. These boards communicate via CAN-FD to synchronize their clocks and coordinate recording and playback states.

## 2. System Architecture
- **Topology**: Daisy-chained CAN-FD bus connecting all 6 motor drivers.
- **Power**: 24V DC bus distributed to all drivers.
- **Control**: Distributed control. One node acts as the "Master" (Clock Source), others are "Slaves".
- **Master Node**: Typically the Base node (ID 0). Responsible for broadcasting the system time and global state (Idle, Recording, Playback).
- **Slave Nodes**: Sync their local clocks to the Master. Report their status and listen for state change commands.

## 3. Hardware Abstraction
Based on the `mnb-ultralight-n17-r1.5` netlist and `BoardConfig.h`:

### 3.1. CAN-FD Interface
- **Controller**: MCP2518FDT-E/QBB
- **Interface**: SPI1 (Hardware SPI)
- **Pins**:
  - **CS**: GPIO 9
  - **SCK**: GPIO 10
  - **MOSI**: GPIO 11
  - **MISO**: GPIO 8
  - **INT**: GPIO 13
- **Transceiver**: Integrated/External (connected to MCP2518)

### 3.2. Flash Storage
- **Chip**: W25Q16JVUXIQ (16M-bit / 2MB)
- **Interface**: QSPI (Standard RP2040 Boot Flash)
- **Usage**:
  - **Bootloader/Firmware**: ~500KB
  - **Filesystem (LittleFS)**: ~1.5MB for storing recording data.

### 3.3. Motor & Encoder
- **Driver**: TMC2240 (SPI0: CS 5, SCK 6, MOSI 7, MISO 4)
- **Encoder**: AS5047 (PIO SPI: CS 19, SCK 20, MOSI 22, MISO 21)

## 4. Synchronization Mechanism
To ensure all joints play back moves in perfect unison, they must share a common time base.

- **Clock Source**: Master Node sends a `SYNC` message every 100ms (10Hz).
- **Message Content**: `uint32_t timestamp_us` (Microseconds since boot/reset).
- **Slave Behavior**:
  - On receipt, Slave updates its local offset: `offset = master_timestamp - local_timestamp`.
  - `corrected_time = local_timestamp + offset`.
  - A simple P-controller or PLL can smooth out jitter if needed, but direct offset correction is likely sufficient for <1ms sync accuracy.

## 5. Recording Workflow
1.  **Trigger**: User presses the "Record" button (Endstop Input) on *any* joint.
2.  **Broadcast**: The node detecting the press broadcasts a `STATE_CHANGE(RECORDING_START)` message.
3.  **State Transition**:
    - All nodes receive the message.
    - **Motor Control**: Switch to "Gravity Compensation" or "Low Current" mode (e.g., 20-30% hold current) to allow manual manipulation.
    - **Logging**: Start buffering `(timestamp, angle)` tuples to RAM.
4.  **Data Rate**: 50Hz - 100Hz sampling.
    - Data point: `uint32_t timestamp` (4B) + `float angle` (4B) = 8 Bytes.
    - 1 minute recording = 60s * 100Hz * 8B = 48KB.
    - RAM Buffer: 64KB circular buffer (plenty of space on RP2040).
5.  **Stop**: User presses the button again.
6.  **Broadcast**: Node broadcasts `STATE_CHANGE(RECORDING_STOP)`.
7.  **Save**:
    - All nodes flush their RAM buffer to the Flash Filesystem (LittleFS).
    - File: `recording_01.bin`.

## 6. Playback Workflow
1.  **Trigger**: User presses the "Playback" button (e.g., Base button or Double-click).
2.  **Broadcast**: Node broadcasts `STATE_CHANGE(PLAYBACK_START)`.
3.  **State Transition**:
    - All nodes load `recording_01.bin` from Flash.
    - **Motor Control**: Switch to "Position Control" (Closed Loop).
4.  **Execution**:
    - Nodes wait for a `START_AT(timestamp)` message from Master to begin moving simultaneously.
    - Each node interpolates its recorded trajectory against the synchronized clock.
    - `target_angle = interpolate(recording, current_synced_time)`.
5.  **Completion**: When the recording ends, nodes hold position and broadcast `PLAYBACK_DONE`.

## 7. Implementation Gap Analysis

### 7.1. Missing Firmware Features
- **CAN Driver**: No library or initialization code for MCP2518FD.
    - *Action*: Add `acan2517FD` or similar library. Configure for SPI1.
- **Flash Filesystem**: No LittleFS integration.
    - *Action*: Add `LittleFS` library. Partition flash in `platformio.ini`.
- **Sync Logic**: No clock synchronization code.
    - *Action*: Implement `ClockSync` class.
- **State Machine**: No high-level state machine (Idle, Record, Playback).
    - *Action*: Implement `SystemStateManager`.
- **Gravity Comp**: Simple "Low Current" mode is easy, but true gravity comp requires a model.
    - *Action*: Start with "Low Current" (e.g., `motor.voltage_limit = 1.0f`).

### 7.2. Required Code Changes
1.  **`platformio.ini`**:
    - Add CAN library (e.g., `pierremolinaro/ACAN2517FD`).
    - Add Filesystem support (`board_build.filesystem = littlefs`).
2.  **`BoardConfig.h`**:
    - Define CAN pins (GPIO 9, 10, 11, 8, 13).
3.  **`lib/CAN`**:
    - Create a wrapper for sending/receiving `SYNC`, `STATE`, and `EVENT` messages.
4.  **`src/main.cpp`**:
    - Initialize CAN.
    - Initialize LittleFS.
    - Add `SystemStateManager` loop.
    - Integrate Button (Endstop) interrupts to trigger state changes.

## 8. Next Steps
1.  **Verify Hardware**: Confirm MCP2518FD is responsive on SPI1.
2.  **Enable CAN**: Get basic communication working between two boards.
3.  **Implement Sync**: Verify timing accuracy.
4.  **Implement Recording**: Test writing to Flash.
5.  **Implement Playback**: Test motion reproduction.
