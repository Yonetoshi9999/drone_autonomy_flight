# Autonomy State Machine Design

## Overview

This document describes the state machine design for autonomous flight operations in Mode 99. The system implements bidirectional communication between the Raspberry Pi and the flight controller at 10Hz.

## System Architecture

```
┌─────────────────────┐          MAVLink (10Hz)          ┌─────────────────────┐
│  Flight Controller  │ ◄──────────────────────────────► │   Raspberry Pi      │
│                     │                                   │                     │
│  - Mode: 99         │   Send: FC Mode & State          │  - State Manager    │
│  - FC States        │   Recv: RPI State                │  - RPI States       │
└─────────────────────┘                                   └─────────────────────┘
         │                                                          │
         │                                                          │
         ▼                                                          ▼
   State Machine                                            State Machine
```

## Communication Protocol

### 10Hz Bidirectional Communication

1. **Receive from Flight Controller** (10Hz):
   - Mode number (Mode 99 = Autonomy)
   - Flight Controller State (PLANNING, INITIALIZING, EXECUTING, IDLE)

2. **Send to Flight Controller** (10Hz):
   - Raspberry Pi State (NOT_SET, ROUTE_SET, EXECUTING, COMPLETED, ERROR)

### MAVLink Message Format

**Receiving from FC:**
- Message Type: `COMMAND_LONG`
- `param1`: Mode (99 for autonomy)
- `param2`: State (1=PLANNING, 2=INITIALIZING, 3=EXECUTING, 4=IDLE)

**Sending to FC:**
- Message Type: `COMMAND_LONG` with `MAV_CMD_USER_1` (31010)
- `param1`: Raspberry Pi State (0=NOT_SET, 1=ROUTE_SET, 2=EXECUTING, 3=COMPLETED, 99=ERROR)

## State Definitions

### Flight Controller States (Received)

| State | Value | Description |
|-------|-------|-------------|
| `PLANNING` | 1 | Request RPI to plan route and set waypoints |
| `INITIALIZING` | 2 | Prepare for first route execution |
| `EXECUTING` | 3 | Active autonomous flight in progress |
| `IDLE` | 4 | End autonomous flight, return to standby |
| `UNKNOWN` | 99 | Unknown or invalid state |

### Raspberry Pi States (Sent)

| State | Value | Description |
|-------|-------|-------------|
| `NOT_SET` | 0 | Default state, no route configured |
| `ROUTE_SET` | 1 | Route planning complete, ready to fly |
| `EXECUTING` | 2 | Autonomous flight in progress |
| `COMPLETED` | 3 | Mission completed successfully |
| `ERROR` | 99 | Error occurred during operation |

## State Transition Diagram

```
Flight Controller (FC)           Raspberry Pi (RPI)              Actions
─────────────────────           ──────────────────              ───────

     [Mode 99]
         │
         ▼
   ┌──────────┐
   │ PLANNING │                   NOT_SET                   Execute waypoint
   └──────────┘                      │                      planning function
         │                           │                            │
         │                           ▼                            ▼
         │                      ROUTE_SET  ◄───────────── [Set destination
         │                           │                      & waypoints]
         ▼                           │
  ┌──────────────┐                  │
  │ INITIALIZING │                  │                      Begin first route
  └──────────────┘                  │                      set to destination
         │                           ▼
         │                      EXECUTING
         │                           │
         ▼                           │
   ┌───────────┐                     │                      Begin autonomy
   │ EXECUTING │ ◄───────────────────┘                     flight to
   └───────────┘                     │                      destination
         │                           │
         │                           ▼                      Continue flight,
         │                      [Flying...]                 monitor progress
         │                           │
         │                           │
         ▼                           ▼
    ┌──────┐                    COMPLETED
    │ IDLE │                         │                      Finish autonomy
    └──────┘                         │                      flight
         │                           ▼
         └──────────────────►    NOT_SET                    Transition back
                                                            to standby
```

## State Transition Rules

### 1. PLANNING → ROUTE_SET

**Trigger:** FC sends Mode=99, State=PLANNING
**Condition:** RPI is in NOT_SET state
**Action:**
- Execute `_execute_waypoint_planning()`
- Calculate destination and waypoints
- Validate waypoints (NFZ check, battery check)
- Send waypoints to FlightController
- Transition to ROUTE_SET

**RPI State Change:** `NOT_SET` → `ROUTE_SET`

### 2. INITIALIZING → EXECUTING

**Trigger:** FC sends Mode=99, State=INITIALIZING
**Condition:** RPI is in ROUTE_SET state
**Action:**
- Execute `_initialize_route()`
- Reset waypoint index to 0
- Prepare for flight execution

**RPI State Change:** `ROUTE_SET` → `EXECUTING`

### 3. EXECUTING (Start)

**Trigger:** FC sends Mode=99, State=EXECUTING (first time)
**Condition:** RPI is in EXECUTING state, mission not started
**Action:**
- Execute `_start_autonomous_flight()`
- Begin waypoint following
- Set mission_started flag

**RPI State Change:** Remains `EXECUTING`

### 4. EXECUTING (Continue)

**Trigger:** FC sends Mode=99, State=EXECUTING (ongoing)
**Condition:** RPI is in EXECUTING state, mission started
**Action:**
- Execute `_continue_autonomous_flight()`
- Monitor progress
- Check if all waypoints reached
- If completed: transition to COMPLETED

**RPI State Change:** `EXECUTING` → `COMPLETED` (when done)

### 5. IDLE → NOT_SET

**Trigger:** FC sends Mode=99, State=IDLE
**Condition:** RPI is not in NOT_SET state
**Action:**
- Execute `_finish_autonomous_flight()`
- Clear waypoints and destination
- Reset mission state

**RPI State Change:** Any state → `NOT_SET`

## Fallback Behavior

### Message Loss Handling

If MAVLink messages are not received:
- **Timeout:** 500ms
- **Behavior:** Use previous FC mode and state
- **Rationale:** Prevents sudden state changes due to temporary communication loss

**Implementation:**
```python
if time.time() - self.last_receive_time < self.message_timeout:
    return StateMessage(
        mode=self.prev_fc_mode,
        state=self.prev_fc_state,
        timestamp=self.last_receive_time
    )
```

## Integration with Control Loops

The system runs three independent asyncio loops:

### 1. Main Control Loop (100Hz / 10ms)
- Sensor data acquisition
- Obstacle detection
- Trajectory planning
- MAVLink command transmission (position + velocity)

### 2. State Management Loop (10Hz / 100ms)
- Receive FC state messages
- Execute state transitions
- Send RPI state messages
- Monitor mission progress

### 3. Photo Capture Loop (2Hz / 500ms)
- Camera capture
- Image storage

## Example Mission Flow

```
Time   FC Mode/State           RPI State       Action
────   ─────────────           ─────────       ──────
T0     -                       NOT_SET         [System idle]

T1     99/PLANNING             NOT_SET         FC requests route planning
T2     99/PLANNING             NOT_SET         RPI calculates waypoints
T3     99/PLANNING             ROUTE_SET       Route planning complete

T4     99/INITIALIZING         ROUTE_SET       FC requests initialization
T5     99/INITIALIZING         EXECUTING       RPI prepares for flight

T6     99/EXECUTING            EXECUTING       FC starts execution
T7     99/EXECUTING            EXECUTING       Autonomous flight begins
...    99/EXECUTING            EXECUTING       [Flying to waypoints]
T20    99/EXECUTING            EXECUTING       Waypoint 1 reached
T40    99/EXECUTING            EXECUTING       Waypoint 2 reached
...    99/EXECUTING            EXECUTING       [Continue flying]
T100   99/EXECUTING            COMPLETED       All waypoints reached

T101   99/IDLE                 COMPLETED       FC requests end
T102   99/IDLE                 NOT_SET         Mission complete, standby
```

## Error Handling

### Route Planning Failure

**Scenario:** Waypoints intersect No-Fly Zone or insufficient battery

**Response:**
- RPI State → `ERROR`
- Print error message with reason
- Wait for FC to send IDLE to reset

### Communication Loss

**Scenario:** No messages received for > 500ms

**Response:**
- Continue using last known FC state
- Log warning if timeout persists > 5 seconds
- Maintain current RPI state

### Mid-Flight Error

**Scenario:** Error during EXECUTING state

**Response:**
- RPI State → `ERROR`
- Flight controller should detect ERROR state
- FC transitions to IDLE to reset

## Implementation Files

- **`autonomy_state.py`**: Core state machine implementation
- **`main.py`**: Integration into main system
- **`flight_controller.py`**: Waypoint management and trajectory control

## Testing

### Unit Tests

Test state transitions:
```python
def test_planning_to_route_set():
    # FC sends PLANNING
    # Verify RPI transitions to ROUTE_SET
    pass

def test_executing_flow():
    # FC sends EXECUTING
    # Verify autonomous flight starts
    pass
```

### Integration Tests

Test with mock flight controller:
```python
def test_full_mission_flow():
    # Simulate complete mission from PLANNING to IDLE
    pass
```

## Configuration

### Timing Parameters

- `state_update_period`: 0.1s (10Hz)
- `message_timeout`: 0.5s
- Control loop: 0.01s (100Hz)

### Autonomy Mode

- Mode number: `99`
- All autonomy operations require Mode=99

## Future Enhancements

1. **Dynamic Replanning**: Support mid-flight route changes
2. **Multi-Mission Support**: Queue multiple missions
3. **Emergency Abort**: Quick transition to safety mode
4. **Extended States**: Add PAUSED, RESUMING states
5. **Telemetry Logging**: Record all state transitions for analysis
