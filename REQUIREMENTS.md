# Smart City Autonomous Rover - Requirements Document

## 1. Project Overview

**Project Type:** Four-wheeled autonomous rover for smart city environment
**Budget:** 1000 RMB maximum
**MCU:** STM32F405RGT6 (LQFP64, Cortex-M4)

## 2. Core Tasks (6 Total)

### Patio 1 - Terrain Navigation
| Task | Description | Key Requirements |
|------|-------------|------------------|
| 1. Lane Tracking | Follow colored path (50cm width) through curves | Color detection, path following |
| 2. Bridge Crossing | Cross 45cm wide, 2.2m long bridge with wire mesh | Traction control, stability |
| 3. Gate Passage | Pass through gate (50cm H x 50cm W), stop at red line | Gate detection, precise stopping |

### Patio 2 - Recognition & Interaction
| Task | Description | Key Requirements |
|------|-------------|------------------|
| 4. Arrow Recognition | Identify arrow shapes, knock down corresponding sign | Shape recognition, path planning |
| 5. Ball Release | Carry ping pong ball to fence, drop into 25cm basket | Positioning, release mechanism |
| 6. Wireless Communication | Send team info via 433MHz HC-12 module | Bluetooth/RF communication |

## 3. Hardware Requirements

### Sensors
- **Camera:** OpenMV (image capturing, lane detection, recognition)
- **Ultrasonic:** SR-04 (obstacle detection)
- **Feedback:** Wheel speed sensor, steering angle sensor
- **Communication:** 433MHz HC-12, Bluetooth module

### Power System
- 12V to 5V: TPS5430 DCDC (for OpenMV, max 3A)
- 5V to 3.3V: LP5907 LDO (for MCU, low ripple)
- Motor supply: TPS5430 to 7-10V

### Motor Driver
- Chip: DRV8837 (built-in MOSFETs, 1.8A output)
- Custom PCB required (no breadboards)

### Safety
- Fuse and ON-OFF switch between battery and system
- Maximum 2 beacons allowed for navigation

## 4. Software Modules Required

### Data Collection
- Camera image capturing
- Ultrasonic distance measurement
- Wheel speed/angle feedback
- Wireless data reception

### Processing & Recognition
- Lane detection and tracking
- Arrow recognition (left/straight/right)
- Color recognition (signals)
- Pedestrian detection
- Obstacle detection

### Decision & Control
- Path planning
- Behavior decision
- Motion planning
- Data fusion
- Steering message generation

### Communication
- UART (OpenMV, Bluetooth)
- 433MHz wireless transmission
- Real-time feedback

## 5. Deliverables

1. **Initial Design Report** - System design, sensor justification (1500 words, 3 pages max)
2. **Laboratory Notebook** - Chronological development record
3. **Final Report** - Team + individual reports, BOM included
4. **Presentation** - 25min PPT + 10min Q&A
5. **Demo Video** - 2 minutes max for non-live patio

## 6. Constraints

- Fully autonomous (pre-programmed, no real-time control transmission)
- Wire connections must be soldered (no breadboards)
- Custom motor driver PCB required
- 12 minutes max for on-site demo
- Tasks must auto-transition (manual reset = score deduction)

## 7. Communication Interfaces

| Interface | Purpose |
|-----------|---------|
| UART1 | Debug output |
| UART2 | OpenMV communication |
| UART3 | Bluetooth module |
| PWM | Servo control |
| GPIO | Motor driver, ultrasonic trigger |
| EXTI | Ultrasonic echo |
