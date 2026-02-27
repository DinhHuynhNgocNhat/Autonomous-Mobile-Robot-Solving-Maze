# Autonomous Mobile Robot - Maze Solver

An autonomous mobile robot based on the Pololu 3pi+ RP2040 that can detect obstacles using Time-of-Flight (ToF) sensors and navigate through mazes.

![Pololu 3pi+ RP2040](https://a.pololu-files.com/picture/0J12019.1000.jpg?cab195d37b5536136110cf943d5e673d)

## Features

- **Obstacle Detection**: Time-of-Flight (ToF) sensors for precise distance measurement
- **Motor Control**: Dual motor control with feedback
- **Encoder Odometry**: Wheel encoders for distance and position tracking
- **OLED Display**: Real-time status and sensor visualization
- **Maze Navigation**: Autonomous pathfinding and obstacle avoidance algorithms

## Hardware

- **Microcontroller**: Raspberry Pi Pico (RP2040)
- **Robot Base**: Pololu 3pi+ RP2040
- **Sensors**: 
  - Time-of-Flight sensors for proximity detection
  - Wheel encoders for odometry
- **Display**: Pololu 128x64 OLED display
- **Motors**: Dual gear motors with wheel encoders
- **Power**: Battery-powered

## Project Structure

```
src/
├── main.cpp          # Main entry point and initialization
├── lab.cpp/h         # Main lab/experiment code
├── motor.cpp/h       # Motor control (PWM and direction)
├── encoder.cpp/h     # Wheel encoder reading and processing
├── sensors.cpp/h     # ToF sensor interfaces
├── oled.cpp/h        # OLED display control
├── pins.h            # Pin definitions and configuration
├── font5x7.h         # Font data for text rendering
└── logo.h            # Bitmap data for logos

build/               # Build output (generated)
include/             # Additional header files
lib/                 # Local libraries
test/                # Test files
platformio.ini       # PlatformIO project configuration
```

## Dependencies

- **PlatformIO**: Build system and package manager
- **Arduino Framework**: For Raspberry Pi Pico
- **Pololu OLED Library** (`pololu/PololuOLED@^2.0.0`): OLED display driver

## Build & Upload Instructions

### Requirements

1. [PlatformIO](https://platformio.org/install) installed
2. Raspberry Pi Pico connected via USB

### Build

```powershell
platformio run -e rpipico
```

### Upload to Device

```powershell
platformio run -e rpipico --target upload
```

### Monitor Serial Output

```powershell
platformio device monitor --baud 115200
```

Connect to the robot via USB to view real-time debug output and sensor readings.

## Key Components

### Motor Control (`motor.cpp/h`)
Manages PWM signals and GPIO pins for motor direction and speed control.

### Wheel Encoders (`encoder.cpp/h`)
Processes rotary encoder signals for odometry and distance measurement.

### Sensors (`sensors.cpp/h`)
Interfaces with ToF sensors via I2C for obstacle detection.

### OLED Display (`oled.cpp/h`)
Renders status information and debug data on the 128x64 display.

## Configuration

Edit the following files to customize the robot:

- **`src/pins.h`**: Define GPIO pins for motors, encoders, sensors
- **`src/lab.cpp`**: Main robot behavior and algorithms
- **`src/sensors.cpp`**: Sensor calibration and thresholds

## Getting Started

1. **Clone the repository**:
   ```powershell
   git clone https://github.com/DinhHuynhNgocNhat/Autonomous-Mobile-Robot-Solving-Maze.git
   cd amr-lab
   ```

2. **Build the project**:
   ```powershell
   platformio run -e rpipico
   ```

3. **Connect Pico and upload**:
   ```powershell
   platformio run -e rpipico --target upload
   ```

4. **Monitor output**:
   ```powershell
   platformio device monitor --baud 115200
   ```

## Debugging

- Check serial output with PlatformIO monitor at 115200 baud
- Use OLED display to verify sensor readings
- LED indicators on the robot show operational status

## License

This project is part of AMR laboratory coursework.

## Author

Dinh Huynh Ngoc Nhat