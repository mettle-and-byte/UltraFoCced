# Copycat Robot Arm Design Document

## 1. Overview
The "Copycat" system allows a 6DOF robot arm to record and replay movements without a central control board. Each joint is controlled by an independent "UltraFoCced" driver board. These boards communicate via CAN-FD to synchronize their clocks and coordinate recording and playback states.

## 2. System Architecture
- **Topology**: Daisy-chained CAN-FD bus connecting all 6 motor drivers.
- **Power**: 24V DC bus distributed to all drivers.
- **Control**: Distributed control. One node acts as the "Leader" (Clock Source), others are "Followers".
- **Leader Node**: The node with the **lowest Node ID** on the bus. Responsible for broadcasting the system time and global state.
- **Follower Nodes**: Sync their local clocks to the Leader. Report their status and listen for state change commands.
- **Concurrency (Dual Core)**:
  - **Core 0**: Dedicated to High-Frequency FOC Loop (Motor Control).
  - **Core 1**: Handles CAN Communication, Flash Storage (LittleFS), and System State Management.
  - **Inter-Core Communication (ICC)**:
    - **Real-Time Data** (Target/Current Angle): **Shared Memory** with `volatile`. On RP2350, 32-bit reads/writes are atomic. This is the fastest method with zero blocking.
    - **Events** (Start/Stop/Estop): **RP2350 SIO FIFO**. Core 1 pushes commands, Core 0 pops them (non-blocking) once per loop. This ensures strict ordering and no missed events.

### 3.2. Flash Storage
- **Chip**: W25Q16JVUXIQ (16M-bit / 2MB)
- **Interface**: QSPI (Standard RP2350 Boot Flash)
- **Usage**:
  - **Bootloader/Firmware**: ~500KB
  - **Filesystem (LittleFS)**: ~1.5MB for storing recording data.

### 3.3. Motor & Encoder
- **Driver**: TMC2240 (SPI0: CS 5, SCK 6, MOSI 7, MISO 4)
- **Encoder**: AS5047 (PIO SPI: CS 19, SCK 20, MOSI 22, MISO 21)

## 4. Synchronization & Addressing
To ensure all joints play back moves in perfect unison, they must share a common time base and have unique addresses.

### 4.1. Network Discovery (Heartbeat)
To allow nodes to detect each other (for Auto-ID and Leader Election), every node must be visible.
- **Mechanism**: **Periodic Heartbeat**.
- **Frequency**: 1Hz (every 1000ms).
- **Message**: `HEARTBEAT(node_id, current_state, uptime)`.
- **Purpose**:
  - Allows new nodes to discover existing IDs.
  - Allows Followers to detect if Leader has failed (timeout).
  - Allows Leader to track active Followers.

### 4.2. Automatic ID Assignment (Sequential Power-Up)
To avoid manual ID configuration, nodes assign themselves an ID based on the bus state at startup.
- **Strategy**: **Listen & Increment** with **Random Backoff**.
- **Procedure**:
  1.  **Power Up**: Node boots.
  2.  **Check Config**:
      - If **ID exists** in Flash: Proceed to **Step 4 (Leader Election)**.
      - If **No ID** (Factory State): Proceed to **Step 3 (Discovery)**.
  3.  **Discovery (Auto-ID)**:
      - **Backoff**: Wait a random duration (0-500ms) to prevent race conditions if multiple nodes boot simultaneously.
      - **Listen**: Listen to CAN bus for 2 seconds.
      - **Assignment**:
          - If **No Traffic**: Assign **ID 0**.
          - If **Traffic**: Collect all unique `node_id`s. Assign **ID = Max(Seen IDs) + 1**.
      - **Persistence**: Save ID to Flash immediately.
      - **Transition**: Proceed to **Step 4** (No reboot required).
  4.  **Usage**: User powers up the Base (ID 0), waits 2s, powers up Joint 1 (becomes ID 1), etc.

### 4.3. Automatic Leader Election
The system automatically selects a "Clock Source" (Leader) without manual configuration.
- **Algorithm**: "Lowest Node ID Wins".
- **Startup**:
  1.  Node enters `LISTENING` state.
  2.  Listens for `SYNC` messages for 500ms.
  3.  **Follower Mode**: If `SYNC` received from a **lower ID**:
      - Sync local clock.
      - Start broadcasting `HEARTBEAT`.
  4.  **Leader Mode**: If timeout (no `SYNC` from lower ID):
      - Broadcast `CLAIM_LEADER(my_node_id)`.
      - Start broadcasting `SYNC` (10Hz) and `HEARTBEAT` (1Hz).
- **Conflict Resolution**:
  - If a Leader receives a `CLAIM_LEADER` or `SYNC` from a **lower ID**: It yields, stops sending `SYNC`, and reverts to Follower.
  - If a Leader receives a `CLAIM_LEADER` from a **higher ID**: It ignores it (the higher ID node will see our `SYNC` and yield).

### 4.4. Clock Sync Protocol
- **Leader Duty**: Broadcasts `SYNC` message every 100ms (10Hz).
- **Message Content**: `uint32_t timestamp_us` (Microseconds since boot/reset).
- **Follower Behavior**:
  - On receipt, Follower updates its local offset: `offset = leader_timestamp - local_timestamp`.
  - `corrected_time = local_timestamp + offset`.
  - **Overflow Handling**: The 32-bit microsecond counter wraps every ~71 minutes. Offset calculation must use modulo arithmetic to handle wrap-around correctly.

## 5. Recording Workflow
1.  **Trigger**: User presses a configured button (e.g., `1.18` - Pin 18 on Node 1).
2.  **Broadcast**: The node detecting the press broadcasts a `STATE_CHANGE_REQUEST(RECORDING_START)` message.
3.  **Coordination (Leader)**:
    - Leader receives request.
    - Leader broadcasts `START_RECORDING_AT(future_timestamp)`.
    - `future_timestamp` = `current_time + 500ms`.
4.  **Verification (ACK)**:
    - Nodes receive `START_RECORDING_AT`.
    - Nodes reply with `ACK(RECORDING_READY)`.
    - **Check**: Leader waits 250ms (half of delay). If `ACK`s received from all known Followers:
        - **Success**: Do nothing (Nodes will start at `future_timestamp`).
        - **Fail**: Broadcast `ABORT`.
5.  **State Transition**:
    - Nodes wait until `corrected_time >= future_timestamp`.
    - **Motor Control**: Switch to "Gravity Compensation" or "Low Current" mode.
    - **Logging**: Start buffering data.
6.  **Data Compression**:
    - Deadband Threshold: `if (abs(current - last) > delta) log()`.
    - Keep-Alive: Log every 1s regardless.
7.  **Stop**: User presses button again.
8.  **Broadcast**: `STATE_CHANGE_REQUEST(RECORDING_STOP)`.
9.  **Save**: Flush to `recording_01.bin`.

## 6. Playback Workflow
1.  **Trigger**: User presses a configured button (e.g., `0.18` - Pin 18 on Base).
2.  **Broadcast**: `STATE_CHANGE_REQUEST(PLAYBACK_START)`.
3.  **Coordination**:
    - Leader broadcasts `START_PLAYBACK_AT(future_timestamp)`.
    - `future_timestamp` = `current_time + 500ms`.
4.  **Verification (ACK)**:
    - Nodes receive `START_PLAYBACK_AT`.
    - Nodes check file existence and reply `ACK(PLAYBACK_READY)`.
    - **Check**: Leader waits 250ms (half of delay). If `ACK`s received from all known Followers:
        - **Success**: Do nothing.
        - **Fail**: Broadcast `ABORT`.
5.  **Execution**:
    - Load `recording_01.bin`.
    - Wait for start time.
    - **Interpolation**: Linear Interpolation between recorded points.
    - **Control**: `motor.move(target_angle)`.
6.  **Completion**: Broadcast `PLAYBACK_DONE`.

## 7. Safety & Error Handling

### 7.1. Heartbeat Monitoring (ESTOP)
- **Leader Responsibility**:
  - Monitors `HEARTBEAT`s from all known Followers.
  - If a known Follower stops sending heartbeats for >2 seconds:
    - Broadcast `ESTOP(REASON_TIMEOUT)`.
- **Follower Responsibility**:
  - Monitors `SYNC` or `HEARTBEAT` from Leader.
  - If Leader stops sending for >2 seconds:
    - Enter `SAFE_STATE` (Gravity Comp).
    - Broadcast `ESTOP(REASON_LEADER_LOST)`.
- **Action on ESTOP**:
  - All nodes immediately switch to **Gravity Compensation / Low Current** mode.
  - LED indicates **Error** state.

### 7.2. Playback Validation
- **Problem**: Recording made with 6 nodes, but only 5 are currently online.
- **Solution**:
  - **Recording Header**: The `recording_01.bin` file starts with a header listing all `node_id`s involved in the recording.
  - **Pre-Flight Check**:
    - When `PLAYBACK_START` is requested, Leader checks the header.
    - Leader verifies all listed nodes are currently sending `HEARTBEAT`s.
    - **Pass**: Broadcast `START_PLAYBACK_AT`.
    - **Fail**: Broadcast `ERROR(MISSING_NODE)`.

### 7.3. LED Status Indicators
The MCU Status LED provides visual feedback on the system state.
- **Leader (Normal)**: Double flash every 2s (Flash-Flash... Pause).
- **Follower (Synced)**: Single flash every 2s, synchronized with Leader's cycle.
- **Unsynced / Discovery**: Slow breathing or steady on.
- **Error**: Fast continuous flashing (5Hz).

## 8. Configuration & Inputs
Configuration is stored locally in `config.ini`.
- **Scope**: `config.ini` only refers to **local** pins on that specific board.
- **Multiple Inputs**: Supported via named sections (e.g., `[input:0]`, `[input:1]`).
- **Example `config.ini`**:
  ```ini
  [node]
  id = 1

  [motor]
  current_limit = 2.0
  hold_current = 0.5

  [input:0]
  # Pin 18 (Endstop)
  pin = 18
  # Actions: TOGGLE_RECORDING, TOGGLE_PLAYBACK, ESTOP
  action = TOGGLE_RECORDING

  [input:1]
  # Another button on Pin 19
  pin = 19
  action = ESTOP
  ```

## 8. Final Design Considerations

### 8.1. "Move to Start" Safety
- **Risk**: If the arm is at 90° and the recording starts at 0°, starting playback immediately will cause a violent snap.
- **Mitigation**: **Homing Phase**.
  - When `PLAYBACK_START` is received, the Leader does *not* broadcast `START_PLAYBACK_AT` immediately.
  - Instead, it broadcasts `MOVE_TO_START(duration_ms)`.
  - Each node calculates a trajectory from `current_angle` to `recording_start_angle` over `duration_ms`.
  - Once all nodes report `AT_START`, the Leader proceeds with `START_PLAYBACK_AT`.

### 8.2. Node Replacement Strategy
- **Problem**: If Node 3 dies and is replaced, the new node might auto-assign ID 6 (next available), breaking the recording.
- **Solution**: **Manual ID Override**.
  - Command: `set_node_id <current_id> <new_id>`.
  - Procedure: User connects to the new node (ID 6), sends `set_node_id 6 3`, and reboots.

### 8.3. CAN Priority Scheme
- **Mechanism**: CAN-FD Arbitration ID (11-bit or 29-bit). Lower value = Higher Priority.
- **Structure**: `(Priority << 7) | NodeID`.
  - **Priority 0 (Highest)**: `ESTOP`, `HEARTBEAT` (Critical Safety).
  - **Priority 1**: `SYNC` (Timing).
  - **Priority 2**: `STATE_CHANGE` (Command & Control).
  - **Priority 3-6**: Reserved.
  - **Priority 7 (Lowest)**: `LOGGING`, `DEBUG` (Bulk Data).

### 8.4. Clock Smoothing
- **Problem**: Hard resetting the local clock on every `SYNC` can cause `dt` jumps, leading to velocity spikes in the FOC loop.
- **Solution**: **Slewed Offset**.
  - Instead of `offset = new_offset`, use `offset += (new_offset - offset) * 0.1`.
  - This filters out jitter and ensures time changes are gradual.

### 8.5. Recording Export & Recovery
- **Use Case**: Backup all recordings to PC, or restore a recording to a replaced node.
- **Export (Dump)**:
  - User sends `dump_all` to any node via USB (The "Sink").
  - Sink sequentially requests data from each Node ID (0-5).
  - Target Node streams `DATA_CHUNK` messages (512 bytes) over CAN.
  - Sink forwards data to USB (Hex/Base64).
- **Import (Restore)**:
  - User sends `restore_node <node_id> <hex_data>` to Sink via USB.
  - Sink streams `DATA_CHUNK` messages to Target Node.
  - Target Node writes to Flash.
- **Note**: This allows recovering a specific joint's motion profile after hardware replacement.

## 9. Implementation Gap Analysis

### 9.1. Missing Firmware Features
- **CAN Driver**: No library or initialization code for MCP2518FD.
    - *Action*: Add `acan2517FD` or similar library. Configure for SPI1.
- **Flash Filesystem**: No LittleFS integration.
    - *Action*: Add `LittleFS` library. Partition flash in `platformio.ini`.
- **Sync Logic**: No clock synchronization code.
    - *Action*: Implement `ClockSync` class.
- **State Machine**: No high-level state machine.
    - *Action*: Implement `SystemStateManager`.
- **Auto-ID Logic**: Need startup listener logic.
    - *Action*: Implement `AutoIDManager` class.
- **Config Parser**: Need INI parser + Remote Config handler.
    - *Action*: Add `inih` and `ConfigManager` class.
- **Safety Monitor**: Heartbeat tracker and ESTOP logic.
    - *Action*: Implement `SafetyMonitor` class.
- **Dual Core**: Need to split tasks between cores.
    - *Action*: Use `rp2040.fifo` or `mutex` for inter-core communication.

### 9.2. Required Code Changes
1.  **`platformio.ini`**:
    - Add CAN library, LittleFS, `inih`.
2.  **`BoardConfig.h`**:
    - Define CAN pins.
3.  **`lib/CAN`**:
    - Create wrapper for `SYNC`, `STATE`, `SET_CONFIG`, `HELLO`, `HEARTBEAT`, `ESTOP`, `ACK`, `ABORT` messages.
4.  **`src/main.cpp`**:
    - Initialize CAN, LittleFS.
    - Run Auto-ID check on first boot.
    - Load Config.
    - Setup Multicore (Core 1 for CAN/Flash).
    - Add `SystemStateManager` loop.
    - Add LED status update loop.

## 10. Next Steps
1.  **Verify Hardware**: Confirm MCP2518FD is responsive on SPI1.
2.  **Enable CAN**: Get basic communication working between two boards.
3.  **Implement Playback**: Test motion reproduction.
