# Development Plan - Smart City Autonomous Rover

## Current Phase: Foundation Development

---

## Phase 1: Hardware Foundation (Current)
- [x] STM32F405 project setup
- [x] Ultrasonic sensor (SR-04) driver
- [x] Kalman filter for distance
- [x] Basic UART communication
- [ ] Motor driver implementation (DRV8837)
- [ ] PWM servo control
- [ ] Power system integration

## Phase 2: Sensor Integration
- [ ] OpenMV camera UART protocol
- [ ] Wheel speed encoder reading
- [ ] Steering angle feedback
- [ ] Bluetooth module (HC-12 433MHz)

## Phase 3: Core Algorithms
- [ ] Lane detection and tracking
- [ ] PID controller for motor
- [ ] Path following algorithm
- [ ] Obstacle avoidance logic

## Phase 4: Recognition Systems
- [ ] Arrow shape recognition
- [ ] Color recognition (signals)
- [ ] Pedestrian detection
- [ ] Gate detection

## Phase 5: Task Implementation
- [ ] Patio 1 Task 1: Lane tracking
- [ ] Patio 1 Task 2: Bridge crossing
- [ ] Patio 1 Task 3: Gate passage
- [ ] Patio 2 Task 4: Arrow recognition
- [ ] Patio 2 Task 5: Ball release
- [ ] Patio 2 Task 6: Wireless communication

## Phase 6: Integration & Testing
- [ ] Data fusion module
- [ ] State machine for task flow
- [ ] Auto-transition between tasks
- [ ] Full system testing

---

## Immediate Next Steps

### Priority 1 (Critical)
1. Implement motor control (PWM + DRV8837)
2. Add PID controller for speed/steering
3. Setup OpenMV communication protocol

### Priority 2 (Important)
4. Lane detection algorithm
5. Basic path following
6. Obstacle avoidance with ultrasonic

### Priority 3 (Enhancement)
7. Arrow/color recognition
8. Communication module
9. Task state machine

---

## Notes
- Update this file after each development session
- Mark completed items with [x]
- Add new tasks as discovered
