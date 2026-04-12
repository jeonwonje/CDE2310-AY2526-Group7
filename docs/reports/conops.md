# CDE2310 — AY25/26 Warehouse Delivery Mission

# CONCEPT OF OPERATIONS (CONOPS)

## Deterministic Ping Pong Ball Launcher System

| Field | Value |
|---|---|
| Group | 7 |
| Document Version | 1.0 |
| Date | 14 Feb 2026 |
| Module | CDE2310 |
| Semester | AY25/26 Semester 2 |

Prepared in accordance with:
- G1 Engineering Design Principles (Jan 2026) — Steps 1 through 5
- NASA Systems Engineering Handbook — Appendix S: ConOps Annotated Outline

---

## 1.0 Introduction

This Concept of Operations (CONOPS) document describes the development of a deterministic ping pong ball launcher system for the CDE2310 AY25/26 Warehouse Delivery Mission. It follows the V-Model design process (Steps 1 through 5) as outlined in the G1 Engineering Design Principles lecture and is structured in accordance with the NASA Systems Engineering Handbook Appendix S ConOps annotated outline.

### 1.1 Project Description

#### 1.1.1 Background

The AY25/26 Warehouse Delivery Mission requires each team to design and build an autonomous or semi-autonomous system capable of delivering standard 40 mm ping pong balls to designated receptacle stations within a mock warehouse environment. The mission comprises two distinct delivery challenges:

- **Station A:** Deliver 3 balls to a stationary receptacle with specific timing intervals between each delivery (4-second gap after the first ball, 6-second gap after the second ball).
- **Station B:** Deliver balls to a moving receptacle mounted on a motorised linear rail, requiring the system to detect AprilTag ID 3 on the rail carriage and fire reactively.

The need for this system arises from the requirement to demonstrate competence in systematic engineering design, mechatronic integration, and deterministic control. The system must achieve reliable, repeatable ball delivery with gentle exit velocities in the range of 1–2 m/s.

#### 1.1.2 Assumptions and Constraints

- **A1:** The playing field dimensions, station positions, and receptacle geometries are as defined in the mission brief on the AY25-26 GitHub repository.
- **A2:** Standard 40 mm table-tennis balls (approximately 2.7 g) will be used.
- **C1:** All structural and mechanical components shall be manufacturable via FDM 3D printing with standard PLA/PETG filament.
- **C2:** Total system cost shall not exceed the allocated project budget.
- **C3:** The system shall comply with all safety regulations specified in the mission brief.
- **C4:** The launcher shall be powered entirely by the TurtleBot3 Burger's onboard Raspberry Pi 4B GPIO (no external Arduino or additional microcontroller).
- **C5:** The system footprint shall not exceed 281 × 306 × 300 mm to remain compatible with the TurtleBot3 base.

### 1.2 Overview of the Envisioned System

#### 1.2.1 System Overview

The envisioned system is a spring-loaded ball launcher mechanism mounted on a TurtleBot3 Burger (281 × 306 × 141 mm base). The launcher uses a rack-and-pinion plunger-and-barrel assembly actuated by an SG90 continuous-rotation servo to compress and release a spring, propelling balls with deterministic, repeatable force. A 3-ball carousel/gravity-fed tube magazine feeds balls into the 42.5 mm inner-diameter barrel, and the RPi 4B running ROS 2 Humble sequences the launch timing and, for Station B, coordinates delivery with the moving receptacle via AprilTag vision (RPi Camera V2, tag36h11 family).

#### 1.2.2 System Scope

The project scope encompasses the mechanical launcher subsystem (barrel, plunger, spring mechanism), the ball-feed subsystem (carousel magazine), the control and actuation subsystem (RPi 4B, SG90 servo, RPi Camera V2), and the integration platform (TurtleBot3 Burger with 3D-printed mounts). The project does not encompass the warehouse field infrastructure, the receptacle stations themselves, or the motorised rail for Station B, as these are provided by the mission organisers.

---

## 2.0 Step 1 — Problem Definition & Requirements

Following the G1 design process, the first step is to rigorously define the problem before implementing any solution. This section establishes the Needs, Goals, and Objectives (NGOs), the system requirements, and the corresponding Measures of Performance (MOPs) and Technical Performance Measures (TPMs).

### 2.1 Needs, Goals, and Objectives

**Need:** A system that can reliably deliver ping pong balls to both stationary and moving receptacle stations in a simulated warehouse environment.

**Goal:** Achieve 100% delivery success rate at Station A and the highest feasible hit rate at Station B during the mission demo.

**Objectives:**
- Achieve a total launch cycle time of ≤ 0.87 seconds per ball.
- Maintain gentle exit velocity in the 1–2 m/s range.
- Operate within the TurtleBot3 Burger size/weight envelope (281 × 306 × ~300 mm total with payload).
- Require no human intervention once the mission begins.
- Complete all deliveries within the 25-minute mission window.

### 2.2 System Requirements

Requirements are written as unambiguous, singular, verifiable "shall" statements in accordance with good requirements practice (G1 lecture — Characteristics of Good Individual System Requirements).

| Req. ID | Requirement Statement | MOP/TPM | Verification |
|---|---|---|---|
| SYS-001 | The system shall deliver 3 balls to Station A within 15 seconds total cycle time (including 4 s and 6 s inter-delivery gaps). | Cycle time ≤ 15 s | Test |
| SYS-002 | The system shall achieve a ball exit velocity of 1–2 m/s (±0.2 m/s). | Measured velocity via high-speed video | Test |
| SYS-003 | The system shall deliver balls to Station B's moving receptacle with ≥ 67% hit rate (2 out of 3 balls). | Hit rate % | Test / Demo |
| SYS-004 | The system shall fit within a footprint of 281 × 306 × 300 mm (on TurtleBot3 base). | Measured dimensions | Inspection |
| SYS-005 | All 3D-printed components shall be manufacturable on a standard FDM printer with ≤ 0.3 mm tolerance compensation. | Dimensional accuracy | Inspection |
| SYS-006 | The launcher shall fire one ball per cycle in ≤ 0.87 seconds. | Time per cycle | Test |
| SYS-007 | The magazine shall hold a minimum of 3 balls without manual reloading. | Ball capacity count | Inspection |
| SYS-008 | The system shall detect AprilTag ID 3 (tag36h11 family) on the Station B rail and fire reactively with a minimum 4-second cooldown between shots. | Detection latency, cooldown enforcement | Test |
| SYS-009 | The launcher shall be controlled entirely via RPi 4B GPIO (pin 12, 50 Hz PWM) without an external microcontroller. | Hardware inspection | Inspection |

*Table 1: System Requirements Matrix.*

### 2.3 Stakeholder Identification

| Stakeholder | Role | Expectations |
|---|---|---|
| NUS EDIC Panel (Module Instructors) | Mission authority, evaluation panel | Mission success rate, design rigour, adherence to V-model process, safety compliance |
| Group 7 Engineering Team | Designers, builders, and operators | Feasibility, integration risk, manageable schedule, clear work division |
| Jeon Won Je | Systems lead, delivery server, CAD assembly | System-level coherence, reliable delivery logic, clean integration |
| Kumaresan | Navigation, frontier exploration, mission coordination | Robust autonomous exploration, accurate goal selection |
| Clara Ong | Perception, launcher node | Reliable AprilTag detection, reactive firing logic |
| Shashwat Gupta | Docking system, launcher mechanism | Precise docking alignment, reliable mechanical launcher |
| Daniel Yow | Integration, CAD (launcher assembly, mounts) | Manufacturable parts, clean mechanical-electrical interfaces |
| Mission Organisers | Field layout, scoring criteria, rail operation | Compliance with mission brief, fair assessment conditions |
| Future CDE2310 Cohorts | Potential re-users of design documentation | Documentation clarity, reproducibility of results |

---

## 3.0 Step 2 — Literature Review / Research

The literature review surveys existing technologies, prior art, and enabling innovations relevant to the launcher design problem. This research informs and constrains the concept generation in Step 3.

### 3.1 Ball Launching Mechanisms

A review of ball-launching mechanisms reveals several established approaches used in recreational, sporting, and educational robotics contexts:

- **Flywheel launchers:** Counter-rotating wheels grip and accelerate the ball. Advantages include continuous feed capability and adjustable speed via wheel RPM. Disadvantages include spin imparted to the ball, inconsistent grip on lightweight 2.7 g ping pong balls, and higher power draw.
- **Spring-loaded / plunger mechanisms:** A compressed spring stores potential energy and releases it to propel the ball through a barrel. Advantages include highly deterministic energy transfer, mechanical simplicity, and low power requirements (only the servo needs power). Disadvantages include single-shot per cycle and the need for a reliable cocking/reset mechanism.
- **Pneumatic launchers:** Compressed air propels the ball. Advantages include smooth acceleration and adjustable force. Disadvantages include the need for a pressure source (pump or pre-charged reservoir), sealing complexity, and difficulty achieving gentle exit velocities in the 1–2 m/s range.
- **Catapult / trebuchet mechanisms:** A lever arm stores energy via torsion spring or counterweight. Advantages include simplicity. Disadvantages include large form factor incompatible with the TurtleBot3 envelope and difficulty achieving repeatable, gentle trajectories.

### 3.2 Feed and Magazine Systems

Ball feed mechanisms reviewed include gravity-fed hoppers with agitator paddles, spring-loaded tube magazines, and rotary indexing drums. For a low-volume application (3 balls), a simple gravity-fed tube magazine with a servo-controlled carousel gate offers the best balance of reliability and simplicity. The carousel indexes one ball at a time into the barrel bore under gravity, ensuring consistent positioning before each shot.

### 3.3 Sensing and Tracking Technologies

For Station B (moving target), potential sensing approaches include:

- **Optical encoders or limit switches** on the motorised rail (if accessible) for position feedback — ruled out as the rail is provided infrastructure and not accessible for sensor mounting.
- **Time-of-flight (ToF) or ultrasonic distance sensors** to detect receptacle position — limited angular resolution and susceptible to reflections in a confined maze.
- **Computer vision with a camera module** for real-time tracking — selected approach. The RPi Camera V2 paired with AprilTag detection (tag36h11 family) provides 6-DOF pose estimation of tag ID 3 mounted on the rail carriage. Detection runs at camera frame rate and triggers reactive firing.
- **Pre-programmed timing** based on characterisation of the rail speed profile (open-loop) — considered as a fallback if vision proves unreliable in testing.

The team selected computer vision (RPi Camera V2 + AprilTag 36h11) as the primary sensing strategy for Station B, with pre-programmed timing as a fallback.

### 3.4 3D Printing and Manufacturability Considerations

Key findings from the literature and prior team experience with FDM printing include the importance of designing adequate clearance for sliding fits (typically 0.3–0.5 mm per side for PLA), accounting for FDM expansion by modelling parts 0.3 mm smaller than target dimensions, and using chamfers at barrel entries and plunger interfaces to prevent binding. Surface finish treatments such as light sanding of barrel interiors can also improve sliding performance — this was confirmed during prototyping when risk R1 (binding) materialised and was resolved by sanding the barrel interior combined with 0.3 mm tolerance compensation in CAD.

### 3.5 Applicable Standards and References

- AY25-26 Warehouse Delivery Mission Brief (GitHub Repository).
- NASA Systems Engineering Handbook, SP-2016-6105 Rev 2.
- G1 Engineering Design Principles lecture notes (Jan 2026).
- TurtleBot3 Burger technical specifications (ROBOTIS e-Manual).
- ROS 2 Humble Hawksbill documentation (docs.ros.org).
- AprilTag: A robust and flexible visual fiducial system — Olson, E. (2011), IEEE ICRA.
- SG90 Micro Servo datasheet (Tower Pro).

---

## 4.0 Step 3 — Concept(s) Design

Following the literature review, the team brainstorms and develops multiple candidate concepts for each subsystem. Each concept is described at a sufficient level of detail to allow comparative evaluation in the BOGAT stage (Step 4).

### 4.1 Concept A — Spring-Loaded Plunger Launcher

This concept uses a linear spring compressed by a servo-driven rack-and-pinion mechanism. The SG90 continuous-rotation servo (GPIO 12, 50 Hz PWM, duty cycle 10.0 for CCW) drives a pinion gear that advances a rack, compressing the spring against the plunger. Upon release, the plunger (40 mm shaft diameter) travels through a 42.5 mm inner-diameter barrel, propelling the 40 mm ball. Key features include:

- 40 mm diameter plunger shaft for centring within the barrel.
- 2.5 mm total clearance between ball and barrel wall (1.25 mm per side).
- Chamfered barrel entry to prevent ball jamming.
- Spring constant and compression distance selected to achieve 1–2 m/s exit velocity.
- Gravity-fed tube magazine (3-ball capacity) with carousel indexing gate to release one ball at a time.
- Complete launch cycle time of 0.87 seconds per ball.

### 4.2 Concept B — Dual-Flywheel Launcher

This concept employs two counter-rotating DC motor-driven wheels. The ball is fed between the wheels via a guide channel. Wheel spacing is set to slightly compress the 40 mm ball for grip. Motor RPM is controlled to regulate exit velocity. A spring-loaded pusher feeds balls from a hopper into the wheel gap. Key concerns include:

- Spin imparted to the lightweight 2.7 g ball causes unpredictable trajectories.
- Inconsistent grip force on a thin-walled, deformable ping pong ball.
- Higher power draw from two DC motors.
- More complex speed control circuitry (ESC or H-bridge per motor).

### 4.3 Concept C — Pneumatic Piston Launcher

This concept uses a small air reservoir and solenoid valve to deliver a controlled air pulse behind the ball seated in the barrel. Pressure is regulated to achieve gentle exit velocity. Key concerns include:

- Requires a pressure source (pump or pre-charged reservoir), adding weight and volume.
- Sealing complexity for consistent pressure delivery.
- Difficulty achieving gentle 1–2 m/s exit velocity without overshooting or undershooting.
- Solenoid valve adds cost and electrical complexity.

### 4.4 Subsystem Concept Summary

| Subsystem | Concept A | Concept B | Concept C | Selected |
|---|---|---|---|---|
| Launcher | Spring plunger (rack-and-pinion servo release) | Dual flywheel (counter-rotating DC motors) | Pneumatic piston (solenoid + air reservoir) | Concept A |
| Feed System | Gravity-fed tube magazine + carousel gate (3-ball) | Spring pusher + hopper | Manual load per shot | Concept A |
| Aiming | Fixed mount on TurtleBot3 + shim adjustment | Pan-tilt servo platform | Rail-mounted sliding bracket | Concept A |
| Sensing (Stn B) | Computer vision (RPi Camera V2 + AprilTag 36h11) | Time-of-flight sensor | Pre-programmed open-loop timing | Concept A (vision) |
| Control | RPi 4B + ROS 2 Humble + GPIO servo control | Arduino + ESC motor control | Arduino + solenoid driver | Concept A (RPi) |

*Table 2: Subsystem Concept Matrix.*

---

## 5.0 Step 4 — BOGAT (Bunch of Guys/Gals Around a Table)

The BOGAT stage is a structured cross-functional review where all team members evaluate the candidate concepts from Step 3. The purpose is to identify blind spots, surface trade-offs across disciplines (mechanical, electrical, software, manufacturing), and converge on the most promising concept for preliminary design.

### 5.1 Evaluation Criteria

The following weighted criteria were used to evaluate concepts. Criteria and weights were agreed upon by all team members before scoring to reduce bias:

| Criterion | Weight | Description |
|---|---|---|
| Determinism / Repeatability | 25% | How consistently can the concept deliver balls to the same spot under identical conditions? |
| Manufacturability | 20% | Ease of fabrication with FDM 3D printing; tolerance sensitivity; assembly complexity. |
| Mechanical Simplicity | 15% | Number of moving parts, failure modes, and maintenance needs. |
| Power / Energy | 10% | Power consumption, battery requirements, and energy efficiency. |
| Controllability (Station B) | 15% | Ease of integrating timing/tracking for the moving target scenario. |
| Cost | 10% | Estimated BOM cost within budget constraints. |
| Safety | 5% | Risk of injury during operation, testing, and transport. |

*Table 3: BOGAT Evaluation Criteria.*

### 5.2 Trade Study / Decision Matrix

Each concept is scored 1–5 against each criterion (5 = best). The weighted total determines the ranking.

| Criterion | Wt. | Concept A (Spring Plunger) | Concept B (Dual Flywheel) | Concept C (Pneumatic) |
|---|---|---|---|---|
| Determinism / Repeatability | 0.25 | 5 (1.25) | 3 (0.75) | 3 (0.75) |
| Manufacturability | 0.20 | 4 (0.80) | 3 (0.60) | 2 (0.40) |
| Mechanical Simplicity | 0.15 | 4 (0.60) | 2 (0.30) | 2 (0.30) |
| Power / Energy | 0.10 | 5 (0.50) | 2 (0.20) | 3 (0.30) |
| Controllability (Stn B) | 0.15 | 4 (0.60) | 4 (0.60) | 3 (0.45) |
| Cost | 0.10 | 5 (0.50) | 3 (0.30) | 2 (0.20) |
| Safety | 0.05 | 4 (0.20) | 3 (0.15) | 2 (0.10) |
| **WEIGHTED TOTAL** | **1.00** | **4.45** | **2.90** | **2.50** |

*Table 4: Decision Matrix.*

### 5.3 BOGAT Outcomes and Rationale

The team reviewed all three concepts with cross-disciplinary input from mechanical (Shashwat, Daniel), software/controls (Jeon, Kumaresan, Clara), and integration perspectives. Key discussion points included:

- **Determinism was the decisive factor.** The spring-plunger mechanism stores a fixed amount of energy (½kx²) and releases it identically each cycle. The flywheel concept's friction-dependent grip on a 2.7 g ball introduced unacceptable variability in exit velocity. The pneumatic concept's sealing tolerances made pressure consistency difficult to guarantee.
- **Manufacturability strongly favoured Concept A.** The barrel, plunger, and rack-and-pinion gears are straightforward FDM prints. The flywheel concept requires precise wheel spacing and balanced motors. The pneumatic concept requires airtight seals — difficult to achieve with FDM.
- **Power budget was a concern.** Concept A requires only a single SG90 servo (powered from RPi GPIO), whereas the flywheel needs two DC motors and ESCs, and the pneumatic needs a pump or pre-charged reservoir.
- **Shashwat raised the binding risk** for Concept A (3D-printed barrel/plunger fit). The team agreed to mitigate this with 0.3 mm FDM tolerance compensation and post-print sanding — a known, manageable risk.

Based on the trade study, the team selects **Concept A (Spring-Loaded Plunger Launcher)** for advancement to Preliminary Design. Key rationale:

1. Highest determinism and repeatability — critical for mission scoring.
2. Simplest mechanical design — fewest moving parts, lowest failure modes.
3. Lowest power consumption — single SG90 servo, no additional motor drivers.
4. Best manufacturability — all components are straightforward FDM prints with known tolerance compensation strategies.

---

## 6.0 Step 5 — Preliminary Design

The preliminary design elaborates the selected concept (Concept A — Spring-Loaded Plunger Launcher) into a detailed engineering definition suitable for review at the Preliminary Design Review (PDR).

### 6.1 System Architecture

The system is decomposed into the following major subsystems:

- **Launcher Subsystem:** A 42.5 mm inner-diameter barrel houses a 40 mm plunger shaft. A compressed spring is loaded via a rack-and-pinion mechanism driven by an SG90 continuous-rotation servo. The servo drives the pinion gear (CCW at duty cycle 10.0), which advances the rack to compress the spring. Upon full compression, the rack disengages and the spring propels the plunger forward, launching the ball. The spring constant and compression distance are tuned to yield 1–2 m/s exit velocity. Chamfered barrel entry prevents ball jamming. FDM tolerance compensation of 0.3 mm is applied to all sliding interfaces.

- **Ball Feed Subsystem:** A 3-ball carousel/gravity-fed tube magazine sits above the barrel. Balls (40 mm diameter, ~2.7 g) are gravity-indexed into the barrel bore one at a time via a carousel gate mechanism. The gate is integrated with the servo actuation cycle — as the rack retracts to cock the spring, the carousel rotates to position the next ball. Magazine capacity of 3 balls is sufficient for both Station A (3 required) and Station B (3 maximum attempts).

- **Control Subsystem:** The Raspberry Pi 4B serves as the sole controller, running ROS 2 Humble. The `rpi_shooter_node` drives the SG90 servo via GPIO pin 12 at 50 Hz PWM. For Station A, the `delivery_server` node executes a timed sequence (fire → 4 s wait → fire → 6 s wait → fire). For Station B, the `delivery_server` subscribes to `/detections` (AprilTagDetectionArray), and upon detecting tag ID 3, calls the `/fire_ball` service with a 4-second cooldown between shots. The RPi Camera V2 feeds `/camera/image_raw` to the `apriltag_detector` node, which performs real-time tag36h11 detection and 6-DOF pose estimation via `cv2.solvePnP`.

- **Structural Platform:** The TurtleBot3 Burger (281 × 306 × 141 mm) serves as the base platform. 3D-printed mounts and guides secure the launcher assembly to the TurtleBot3's top plate. The total system height including the launcher and magazine is approximately 300 mm. Daniel's CAD assembly ensures all mechanical interfaces are dimensionally consistent and printable.

### 6.2 Key Design Parameters

| Parameter | Value | Justification / Traced Req. |
|---|---|---|
| Ball diameter | 40 mm | Standard table-tennis ball spec (A2). |
| Ball mass | ~2.7 g | Standard table-tennis ball spec (A2). |
| Barrel inner diameter | 42.5 mm | 2.5 mm total clearance to prevent jamming (SYS-005). |
| Plunger shaft diameter | 40 mm | Centring within barrel; 0.3 mm FDM compensation applied (SYS-005). |
| Servo model | SG90 continuous rotation | Low cost, sufficient torque for rack-and-pinion cocking. |
| Servo GPIO pin | 12 (BCM) | RPi 4B hardware PWM-capable pin (SYS-009). |
| PWM frequency | 50 Hz | Standard servo control frequency. |
| CCW duty cycle | 10.0 | Calibrated for consistent rack advance speed. |
| Time per launch cycle | 0.87 s | Measured; satisfies SYS-006. |
| Target exit velocity | 1–2 m/s | SYS-002; gentle delivery per mission brief. |
| Magazine capacity | 3 balls | Sufficient for Station A (3 balls) and Station B (3 attempts) (SYS-007). |
| FDM tolerance compensation | −0.3 mm | Modelled parts 0.3 mm undersized to account for FDM expansion (SYS-005). |
| System footprint | 281 × 306 × ~300 mm | On TurtleBot3 Burger base with launcher payload (SYS-004). |
| Station A inter-delivery gaps | 4 s, then 6 s | Per mission brief timing requirements (SYS-001). |
| Station B cooldown | 4.0 s | Minimum wait between reactive shots (SYS-008). |
| Station B target tag | AprilTag ID 3 (tag36h11) | Per mission brief; detected by RPi Camera V2 (SYS-008). |

*Table 5: Key Design Parameters.*

### 6.3 Operational Scenarios

#### 6.3.1 Nominal Operation — Station A (Stationary Target)

The mission coordinator sends `START_DELIVERY` with target `tag36h11:0` to the delivery server. The delivery server executes the static delivery sequence: it calls the `/fire_ball` service on the RPi shooter node to fire the first ball, waits 4 seconds, fires the second ball, waits 6 seconds, then fires the third ball. Each fire call triggers the SG90 servo to rotate CCW (duty 10.0) for 0.87 seconds, which cocks and releases the spring-loaded plunger, launching one ball from the magazine through the barrel. Upon completion of all 3 shots, the delivery server publishes `DELIVERY_COMPLETE` on `/mission_status`. Total Station A delivery time is approximately 11.7 seconds (0.87 + 4.0 + 0.87 + 6.0 + 0.87 s).

#### 6.3.2 Nominal Operation — Station B (Moving Target)

The mission coordinator sends `START_DELIVERY` with target `tag36h11:2` to the delivery server. The delivery server activates dynamic delivery mode and subscribes to `/detections` from the `apriltag_detector` node. The RPi Camera V2 continuously captures frames, which the detector processes for tag36h11 markers. When AprilTag ID 3 (mounted on the moving rail carriage) enters the camera's field of view and is detected, the delivery server calls `/fire_ball` and enters a 4-second cooldown period. During cooldown, further detections are ignored. After cooldown expires, the system is ready for the next reactive shot. This repeats for up to 3 shots. Upon firing 3 balls or upon mission coordinator timeout, `DELIVERY_COMPLETE` is published.

#### 6.3.3 Off-Nominal / Failure Scenarios

- **Ball jam in barrel (R1):** Plunger binds due to FDM tolerance issues. Mitigated by 0.3 mm tolerance compensation in CAD and post-print sanding of barrel interior. During testing, this risk materialised and was successfully resolved with these mitigations.
- **Feed gate failure:** Carousel fails to index next ball. Backup: operator manually loads ball during pre-mission setup if mechanism is unreliable.
- **Sensor failure — Station B (R3):** Camera feed drops or AprilTag detection fails. Fallback: pre-programmed timing mode based on characterised rail speed profile (open-loop). During development, the team confirmed AprilTag vision was reliable and did not need to deploy the fallback.
- **Spring fatigue (R2):** Spring provides inconsistent force after repeated cycles. Mitigated by selecting a quality spring, characterising fatigue behaviour in testing, and carrying a spare spring in the kit.
- **Servo stall / overload:** SG90 servo stalls during rack-and-pinion cocking. The `rpi_shooter_node` enforces a threading lock to prevent concurrent fire commands and a `MAX_BALLS = 3` counter to prevent over-cycling.

---

## 6.4 Preliminary Design Validation

Prior to PDR, the team validated the preliminary design against the system requirements (Table 1) through the following methods:

- **Analytical calculation** of spring energy and exit velocity vs. SYS-002: ½kx² energy budget confirmed to yield 1–2 m/s for the selected spring constant and compression distance, given the 2.7 g ball mass.
- **CAD interference and tolerance stack-up analysis** vs. SYS-004 and SYS-005: Daniel's CAD assembly verified all parts fit within the 281 × 306 × 300 mm envelope with no interferences.
- **Prototype barrel/plunger fit check** via test prints: Initial prints confirmed binding risk (R1). After applying 0.3 mm tolerance compensation and sanding barrel interiors, smooth sliding fit was achieved.
- **Functional firing test:** Measured launch cycle time of 0.87 seconds per ball (SYS-006) and exit velocity in the 1–2 m/s range (SYS-002) using high-speed video analysis.
- **AprilTag detection validation:** Confirmed reliable detection of tag36h11 ID 3 at Station B operating distances using the RPi Camera V2 and `cv2.solvePnP` pose estimation.

---

## 7.0 Risks and Potential Issues

| ID | Risk Description | Likelihood | Impact | Mitigation Strategy | Status |
|---|---|---|---|---|---|
| R1 | 3D-printed parts bind due to insufficient clearance between plunger (40 mm) and barrel (42.5 mm ID). | Medium | High | Design with 0.3 mm FDM compensation; test-print fit checks early; sand barrel interior post-print. | Realised and resolved — sanding + tolerance compensation eliminated binding. |
| R2 | Spring provides inconsistent force over repeated launch cycles due to fatigue. | Low | High | Select quality spring; characterise fatigue behaviour over 50+ cycles; carry spare spring in kit. | Monitored — no degradation observed in testing. |
| R3 | Moving target tracking (Station B) proves unreliable with reactive vision approach. | Medium | Medium | Primary: AprilTag vision (RPi Camera V2 + tag36h11 ID 3). Fallback: pre-programmed open-loop timing based on rail speed characterisation. | Resolved — AprilTag vision selected and validated; open-loop fallback available but not needed. |
| R4 | Budget overrun due to component sourcing delays for servo, spring, or camera module. | Low | Medium | Identify alternative suppliers; order critical items (SG90 servo, RPi Camera V2) early in project timeline. | Mitigated — all components sourced on schedule. |
| R5 | Carousel magazine fails to reliably index balls into barrel bore. | Low | Medium | Gravity-fed design minimises moving parts; test with actual 40 mm ping pong balls early; adjust carousel gate clearances if needed. | Monitored — reliable in testing with 3-ball loads. |

*Table 6: Risk Register.*

---

## 8.0 Summary and Path Forward

This CONOPS document has outlined the problem definition, system requirements, literature review findings, candidate concepts, BOGAT trade-study evaluation, and preliminary design for the CDE2310 Warehouse Delivery Mission ping pong ball launcher system. The selected concept — **Concept A (Spring-Loaded Plunger Launcher)** — provides the best balance of deterministic performance, manufacturability, and controllability as validated through the weighted decision matrix (score: 4.45/5.00, versus 2.90 for dual flywheel and 2.50 for pneumatic).

The system integrates a rack-and-pinion servo-driven spring release, a 3-ball carousel magazine, and RPi Camera V2 AprilTag vision — all controlled by the Raspberry Pi 4B running ROS 2 Humble — mounted on the TurtleBot3 Burger platform within a 281 × 306 × ~300 mm footprint.

**Immediate next steps:**
1. Complete the preliminary design documentation and prepare for the 20-minute Preliminary Design Review (PDR) presentation.
2. Upon PDR approval, proceed to the build/prototype phase (Step 6: Prototyping and Testing) per the V-Model design process.
3. Fabricate all 3D-printed components with validated 0.3 mm tolerance compensation.
4. Integrate launcher assembly onto TurtleBot3 and perform end-to-end firing tests at both Station A and Station B configurations.
5. Conduct full mission rehearsals within the 25-minute window to validate system-level performance.

**Team responsibilities for next phase:**

| Member | Responsibility |
|---|---|
| Jeon Won Je | Systems integration, delivery server node, CAD assembly review |
| Kumaresan | Navigation stack, frontier exploration, mission coordinator |
| Clara Ong | Perception pipeline (AprilTag detector), launcher node integration |
| Shashwat Gupta | Docking server, launcher mechanical assembly and tuning |
| Daniel Yow | CAD finalisation (launcher assembly, TurtleBot3 mounts), 3D printing |
