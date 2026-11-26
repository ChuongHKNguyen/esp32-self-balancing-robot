# ESP32-Based Self-Balancing Robot

<p align="center">
  <strong>Robotics · Embedded Systems · Feedback Control</strong>
</p>

<p align="center">
  <img alt="Platform ESP32" src="https://img.shields.io/badge/Platform-ESP32-E7352C?style=flat-square&logo=espressif&logoColor=white">
  <img alt="Framework Arduino" src="https://img.shields.io/badge/Framework-Arduino-00878F?style=flat-square&logo=arduino&logoColor=white">
  <img alt="Language C++" src="https://img.shields.io/badge/Language-C%2B%2B-00599C?style=flat-square&logo=cplusplus&logoColor=white">
  <img alt="Control PID" src="https://img.shields.io/badge/Control-PID-3568A8?style=flat-square">
  <img alt="Status In development" src="https://img.shields.io/badge/Status-In%20Development-E3A008?style=flat-square">
</p>

<!-- Replace the placeholder URL below with assets/images/robot-hero.jpg after uploading your photograph. -->
<p align="center">
  <img src="images/cover.PNG" alt="Self-Balancing Robot" width="400">
  <br>
  <em>Figure 1. ESP32-based self-balancing robot prototype.</em>
</p>

<p align="center"><em>Two-wheeled robotic platform for real-time orientation estimation and closed-loop stabilization.</em></p>

<p align="center">
  <a href="#demonstration">Demonstration</a> ·
  <a href="#system-architecture">Architecture</a> ·
  <a href="#hardware">Hardware</a> ·
  <a href="#getting-started">Getting Started</a> ·
  <a href="#testing-and-validation">Testing</a>
</p>

---

## Project Overview

This project develops a two-wheeled self-balancing robot based on the ESP32 microcontroller. An MPU6500 inertial measurement unit (IMU) provides accelerometer and gyroscope measurements for estimating the robot's tilt. A proportional–integral–derivative (PID) controller calculates corrective motor commands, which are applied through a dual H-bridge motor driver. An ESP32-hosted web dashboard supports wireless monitoring and controller tuning during development.

## Demonstration

### Robot operation

<!-- Replace BOTH occurrences of YOUR_VIDEO_ID with the YouTube video ID. Alternatively, use a locally uploaded thumbnail as the image source. -->
[![Watch the self-balancing robot demonstration](https://www.youtube.com/shorts/msnDinaH7lM?feature=share)](https://www.youtube.com/shorts/msnDinaH7lM?feature=shareD)

*Click the thumbnail to watch the full demonstration on YouTube once the video is published.*

<!-- After uploading the video, replace the placeholder image URL above with:
https://img.youtube.com/vi/YOUR_VIDEO_ID/hqdefault.jpg
Or use: assets/images/demo-thumbnail.jpg
For a short looping demonstration, upload assets/media/balance-demo.gif and uncomment:
![Balancing demonstration](assets/media/balance-demo.gif)
For inline MP4 playback, edit this README on GitHub, drag the MP4 into the editor, and retain the generated attachment URL here.
-->

## Key Features

- ESP32-based embedded control using Arduino C/C++.
- Six-axis accelerometer and gyroscope measurements from the MPU6500.
- IMU calibration and orientation offset compensation.
- Closed-loop PID stabilization with bidirectional PWM motor control.
- ESP32 Wi-Fi access point and browser-based monitoring interface.
- Adjustable PID gains through the web dashboard.
- Target control-loop frequency of 100 Hz, subject to timing validation.

## System Architecture

```mermaid
flowchart LR
    R[Upright angle reference] --> E((Error))
    E --> P[PID controller]
    P --> D[L298N motor driver]
    D --> M[GA25-370 motors]
    M --> B[Robot dynamics]
    B --> I[MPU6500 IMU]
    I --> F[Orientation estimation]
    F --> E
    W[Web dashboard] <-->|Wi-Fi telemetry and tuning| C[ESP32 web server]
    C -. Gain updates .-> P
    F -. Telemetry .-> C
```

The feedback loop runs on the ESP32. The browser provides a supervisory interface rather than executing the balance controller. The architecture above describes the intended functional design; actual scheduling and timing must be verified against the firmware.

## Hardware

### Bill of materials

| Component | Specification | Quantity | Function |
|---|---|---:|---|
| Microcontroller | ESP32-WROOM-32 development board, 38-pin | 1 | Embedded processing and wireless communication |
| IMU | MPU6500 | 1 | Angular velocity and acceleration sensing |
| Motor driver | L298N dual H-bridge module | 1 | Bidirectional motor actuation |
| Geared DC motor | GA25-370 | 2 | Wheel actuation |
| Wheels | Compatible with motor shaft | 2 | Ground contact |
| Chassis | Prototype frame | 1 | Mechanical support |
| Power supply | To be documented | 1 | Motor and logic power |

<!-- TODO: Add motor voltage, gear ratio, rated speed, stall current, wheel diameter, battery chemistry/voltage, robot mass and dimensions when measured. -->

### Pin assignment

The following is the working pin map carried forward from the earlier development configuration. Verify it against the final sketch and physical wiring before connecting power.

| Device | Signal | ESP32 GPIO | Function |
|---|---|---:|---|
| MPU6500 | SDA | 21 | I²C data |
| MPU6500 | SCL | 22 | I²C clock |
| L298N | ENA | 25 | Motor A PWM enable |
| L298N | IN1 | 26 | Motor A direction |
| L298N | IN2 | 27 | Motor A direction |
| L298N | IN3 | 32 | Motor B direction |
| L298N | IN4 | 33 | Motor B direction |
| L298N | ENB | 14 | Motor B PWM enable |

### Electrical connection overview

```mermaid
flowchart TB
    S[Motor power supply] --> D[L298N module]
    L[Regulated logic supply] --> E[ESP32]
    E -->|GPIO 21 / 22 - I2C| I[MPU6500]
    E -->|PWM and direction GPIOs| D
    D --> A[Left motor]
    D --> B[Right motor]
    G[Common ground] --- E
    G --- D
    G --- I
    G --- S
    G --- L
```

*Functional connection diagram, not a complete electrical schematic.*

- Ensure all logic and motor supply grounds share a suitable common reference.
- Verify the MPU6500 breakout's supply requirements and ensure I²C pull-ups are compatible with 3.3 V ESP32 logic.
- Remove ENA/ENB jumpers if required by the L298N module when using external PWM signals.
- Verify motor stall current, supply voltage, driver thermal limits and voltage drop before applying power. GA25-370 motors exist in multiple variants.
- Do not connect motor supply voltage directly to an ESP32 GPIO or 3.3 V rail. Check the L298N module's regulator jumper configuration before connecting logic power.

<!-- Upload a verified wiring schematic to assets/diagrams/wiring.png and uncomment:
![Electrical wiring schematic](assets/diagrams/wiring.png)
-->

## Control System

### Orientation estimation

The accelerometer provides a gravity-based tilt estimate, while the gyroscope measures angular velocity. Accelerometer measurements can be affected by linear acceleration and vibration; integrating gyroscope measurements introduces drift. Combining these signals can improve the tilt estimate.

For a complementary-filter implementation, the representative discrete-time equation is:

$$
\hat{\theta}_k = \alpha(\hat{\theta}_{k-1} + \omega_k\Delta t) + (1-\alpha)\theta_{\mathrm{acc},k}
$$

where $\hat{\theta}$ is the estimated tilt, $\omega$ is the bias-corrected angular velocity about the balancing axis, $\Delta t$ is the sampling interval, and $\alpha$ is the filter coefficient.

The balancing axis, sign convention and filter coefficient must be matched to the actual IMU mounting and firmware. This equation documents a representative approach, not a verified transcription of the current sketch.

### PID controller

The controller minimizes the difference between the target and estimated tilt:

$$
e(t)=\theta_{\mathrm{ref}}-\hat{\theta}(t)
$$

$$
u(t)=K_p e(t)+K_i\int_0^t e(\tau)\,d\tau+K_d\frac{de(t)}{dt}
$$

The signed control output is mapped to motor direction and PWM magnitude, subject to the configured output limit. Motor polarity must be verified so that the wheels move to counter the robot's fall.

| Parameter | Previous development setting | Purpose |
|---|---:|---|
| $K_p$ | 25 | Proportional correction |
| $K_i$ | 0 | Accumulated error correction |
| $K_d$ | 0.5 | Damping through error-rate feedback |
| PWM limit | 120 | Limit on motor command magnitude |
| Control frequency | 100 Hz | Target loop update rate |
| Calibration samples | 300 | Stationary IMU calibration |

These are historical development settings, not validated optimal gains. Their effective units and PWM scale depend on the final implementation. Confirm values against the current firmware before use.

### Firmware workflow

```mermaid
flowchart TD
    A[Power on] --> B[Initialize peripherals and outputs]
    B --> C[Calibrate stationary IMU]
    C --> D[Start Wi-Fi and web interface]
    D --> E[Read accelerometer and gyroscope]
    E --> F[Estimate tilt and calculate error]
    F --> G[Compute PID output]
    G --> H[Apply output limits and motor commands]
    H --> I[Update telemetry]
    I --> E
```

*Conceptual firmware workflow. Safe-start gating, IMU fault handling and fall detection should be verified or added before untethered testing.*

## Web Dashboard

<p align="center">
  <img src="https://placehold.co/1100x600/182431/E2E8F0.png?text=ESP32+Web+Dashboard%0AScreenshot+Placeholder" alt="Web dashboard screenshot placeholder" width="850">
</p>

<!-- Replace the placeholder with assets/images/dashboard.png. -->

The embedded web interface is intended to display orientation readings and provide adjustment of PID gains during development.

| Network parameter | Previous configuration |
|---|---|
| Wi-Fi mode | Access point |
| SSID | `ESP32_BALANCE` |
| Development password | `12345678` |
| Dashboard address | `http://192.168.4.1` |

Confirm the network settings in the actual sketch. Change the default password before use in shared environments. The dashboard should not be treated as a safety-critical emergency-stop interface.


## Getting Started

### Prerequisites

- Arduino IDE 2.x or another compatible ESP32 development environment.
- Espressif ESP32 Arduino board package.
- USB data cable and appropriate USB-to-serial driver, if required by the development board.
- Any additional IMU or networking libraries imported by the final sketch.

The exact ESP32 board package version and external library dependencies must be recorded after the final firmware is uploaded. ESP32 Arduino core versions can differ in their PWM APIs.

### Installation

1. Clone or download the repository from GitHub. After publishing, replace `YOUR_GITHUB_USERNAME` in the example below:

   ```bash
   git clone https://github.com/YOUR_GITHUB_USERNAME/esp32-self-balancing-robot.git
   cd esp32-self-balancing-robot
   ```

2. Open `firmware/self_balancing_robot/self_balancing_robot.ino` in Arduino IDE.
3. Install the ESP32 board package and any dependencies required by the sketch.
4. Select the board profile appropriate to the ESP32 development board (commonly `ESP32 Dev Module`) and the correct serial port.
5. Verify pin assignments, IMU orientation, motor polarity and power connections.
6. Compile and upload the firmware with motor power disconnected.
7. Verify stationary IMU readings and calibration before enabling the motor supply.
8. Connect to the configured ESP32 Wi-Fi network and open the dashboard address in a browser.

### Initial operation and tuning

1. Secure the robot in a test stand with the wheels clear of the ground and keep a physical power disconnect within reach.
2. Keep the robot stationary in the intended calibration orientation during startup.
3. Verify that the displayed tilt and gyro readings respond correctly to manual movement.
4. With limited motor output, check that both motors respond in the direction required to counter a fall.
5. Tune proportional gain gradually, then derivative gain to improve damping. Introduce integral action only if necessary and with appropriate anti-windup protection.
6. Verify motor cutoff behavior, sensor fault response and output limits before attempting free-standing balance.
7. Perform controlled balancing trials in a clear area with a suitable support or restraint.

## Testing and Validation

The following test plan is proposed to document the prototype's behavior reproducibly. Results have not been supplied and are deliberately left unreported.

| Test | Metric / acceptance criterion | Result |
|---|---|---|
| IMU stationary calibration | Mean tilt offset and gyro bias | Pending documentation |
| Orientation estimation | Response and drift at known orientations | Pending documentation |
| Motor direction | Correct corrective direction for both tilt signs | Pending documentation |
| Control-loop timing | Mean frequency, jitter and missed deadlines | Pending documentation |
| Static balancing | Duration, RMS tilt error and peak deviation | Pending documentation |
| Disturbance rejection | Recovery time and maximum tilt | Pending documentation |
| Dashboard load | Control timing with active browser clients | Pending documentation |
| Fault handling | Motor shutdown on invalid IMU data or excessive tilt | Pending documentation |

### Performance results

<!-- Replace with measured plots and results. Do not report simulated or estimated values as experimental measurements. -->
<p align="center">
  <img src="https://placehold.co/1000x450/243447/E2E8F0.png?text=Experimental+Results%0ATilt+Angle+vs+Time" alt="Experimental plot placeholder — no measured data" width="800">
</p>

*Placeholder for measured tilt-angle time series. Include the target angle, PID gains, sampling rate, test conditions and units when publishing results.*

## Engineering Considerations and Limitations

- **Mechanical dynamics:** Centre of mass, wheel diameter, gearbox backlash, friction and mass distribution influence balancing behavior.
- **Motor driver:** The L298N's voltage drop and thermal performance can limit available motor torque and efficiency.
- **Sensor estimation:** Vibration, acceleration and mounting alignment may affect orientation accuracy.
- **Timing:** Wi-Fi and web-server activity must not introduce excessive control-loop latency or jitter.
- **Position control:** Tilt stabilization alone does not guarantee that the robot maintains a fixed ground position. Wheel-speed or position feedback would be needed for more advanced control.
- **Safety:** Output saturation, integral windup, sensor failure and excessive tilt need explicit handling. Their implementation has not been verified in the current firmware.

## Development Roadmap

- [ ] Publish the current Arduino firmware and record dependency versions.
- [ ] Document the final wiring schematic and mechanical specifications.
- [ ] Validate IMU calibration and balancing-axis orientation.
- [ ] Verify safe motor direction, output limits and fault cutoff.
- [ ] Tune PID gains and establish repeatable balancing tests.
- [ ] Measure control-loop timing under Wi-Fi traffic.
- [ ] Publish experimental plots and demonstration videos.
- [ ] Investigate encoder-based velocity and position feedback, if supported by the motors.
- [ ] Evaluate alternative control strategies and more efficient motor drivers.

## References

- [Arduino ESP32 documentation — Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [Arduino documentation](https://docs.arduino.cc/)
- [L298 dual full-bridge driver — STMicroelectronics](https://www.st.com/en/motor-drivers/l298.html)
- [Inverted Pendulum: System Modeling — University of Michigan, Control Tutorials for MATLAB and Simulink](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)

<!-- Add the exact MPU6500 breakout and GA25-370 variant datasheets, and any libraries or algorithms used in the actual implementation. Credit third-party code and assets where applicable. -->

## Author

**YOUR_NAME**  
Robotics and Mechatronics Engineering

<!-- Replace the placeholders and remove any links you do not want to publish. -->
[GitHub](https://github.com/YOUR_GITHUB_USERNAME) · [LinkedIn](https://www.linkedin.com/in/YOUR_LINKEDIN_ID/) · [YouTube](https://www.youtube.com/@YOUR_CHANNEL)

## License

A project license has not yet been selected. Add the appropriate `LICENSE` file before granting reuse permissions. Third-party dependencies remain subject to their respective licenses.

---

<p align="center"><sub>Designed and developed as a hands-on investigation of embedded robotic stabilization.</sub></p>

<!--
PUBLISHING CHECKLIST (not visible in rendered README)
- Replace YOUR_NAME, YOUR_GITHUB_USERNAME, YOUR_LINKEDIN_ID, YOUR_CHANNEL and YOUR_VIDEO_ID.
- Upload your images and replace all placehold.co URLs with relative paths to the corresponding assets.
- Replace the video placeholder with https://img.youtube.com/vi/YOUR_VIDEO_ID/hqdefault.jpg or a local thumbnail.
- Upload a GIF for looping playback or attach an MP4 via GitHub's Markdown editor for supported inline playback.
- Verify actual motor-driver model (L298N assumed), GPIOs, gains, calibration, Wi-Fi and loop frequency against firmware.
- Confirm which features are implemented; remove or revise unimplemented claims.
- Update the repository tree, dependencies, test results and roadmap.
- Choose a license and review all photographs, videos and source files for private information before publishing.
-->

