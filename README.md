# PID Simulation - Hydraulic Motor

## Description

This project implements a **physical simulation** of a pendulum controlled by a hydraulic motor, utilizing a PID (Proportional, Integral, Derivative) controller to stabilize the system. The simulation includes:

1. **Physical Simulation**:
   - Modeling the behavior of a pendulum subject to gravity, viscous friction, and external torques.
   - Numerical integration using Euler and 4th-order Runge-Kutta methods.

2. **PID Control**:
   - Dynamic adjustment of the hydraulic motor's pressure to maintain the pendulum's angle close to the desired setpoint.

3. **Graphical Interface**:
   - Real-time graphs displaying the histories of angle, error, and pressure.
   - Visual rendering of the pendulum to track the system's movement.
   - Interactive controls for adjusting PID parameters, pressure, and resetting the simulation.

---

## Features

### Version 1.0

1. **Physical Simulation**:
   - Realistic model for a pendulum controlled by a hydraulic motor.
   - Numerical integration to simulate movement over time.

2. **PID Control**:
   - PID controller to keep the pendulum in the vertical position.

3. **Graphical Interface**:
   - Angle and error graphs to monitor control performance.
   - Visual rendering of the pendulum to track movement.

4. **Adjustable Parameters**:
   - Adjustment of mass, length, PID gains, and pendulum setpoint.

### Version 1.1

1. **Stability**:
   - Improvements in system stability with adjustments to PID gains and sampling rate.

2. **Graphical Interface**:
   - Inclusion of error and pressure graphs to monitor control performance.
   - Interactive controls to adjust PID parameters in real-time.

3. **Signal Filtering**:
   - Application of an exponential filter to smooth pressure, reducing abrupt oscillations.

4. **Sensor Noise**:
   - Simulation of sensor noise to add realism to angle measurements.

5. **Reset Functionality**:
   - Option to reset the simulation at any time to test different configurations.

### Version 1.2

1. **Physical Limits**:
   - Added torque saturation to reflect real-world torque limitations.

2. **External Disturbance**:
   - Parameterization of disturbance magnitude and direction for greater control over tests.

3. **PID Gains**:
   - Inclusion of limits on \( K_p \), \( K_i \), \( K_d \) to prevent adjustments that could destabilize the system.

4. **Experimental Validation**:
   - Comments on the need for experimental validation to ensure parameters correspond to the real system.

5. **Documentation**:
   - Detailed explanation of system physics, PID control functionality, and code implementation.

### Version 1.3

1. **Compiler Warning Fixes**:
   - **Implicit Declaration of Function `Clamp`**:
     - Resolved the warning by ensuring the `Clamp` function is properly declared. Included the appropriate header (`raylib.h`) in `main.c`.

2. **Code Cleanup and Maintainability**:
   - **Centralized Color Definitions**:
     - Consolidated color definitions at the beginning of `gui.c` for better maintainability and consistency across the UI.
   - **Descriptive Naming Conventions**:
     - Improved naming conventions for color constants to enhance code readability and understanding.

3. **Modular Code Structure**:
   - **Code Modularity**:
     - Refactored the codebase to be more modular by separating functionalities into distinct files. This enhances readability, maintainability, and scalability of the project.
     - **File Separation**:
       - Organized related functions and definitions into separate source (`.c`) and header (`.h`) files. For example:
         - `gui.c` and `gui.h` handle all graphical user interface components.
         - `pid.c` and `pid.h` manage PID controller logic.
         - `sim.c` and `sim.h` are responsible for the simulation mechanics.
         - `utils.c` and `utils.h` contain utility functions.
         - `log.c` and `log.h` handle data logging functionalities.
       - This separation ensures that each module has a clear responsibility, making the codebase easier to navigate and maintain.

4. **Enhanced Documentation**:
   - Updated `README.md` to reflect recent code improvements and maintenance actions.
   - Added detailed explanations of the modular structure to assist future contributors and maintainers in understanding the project layout.

---

## Requirements

To compile and run this simulation, you will need the following components:

- **C Compiler**: GCC recommended.
- **Raylib**: Library for creating graphical interfaces.
- **Make**: Optional, to facilitate compilation.

### Installing Raylib

#### Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install libraylib-dev
```

#### macOS

Using Homebrew:

```bash
brew install raylib
```

#### Windows

1. Download and install [MinGW](http://www.mingw.org/).
2. Follow the Raylib installation instructions for Windows available in the [official documentation](https://www.raylib.com/).

---

## Compilation

### Step-by-Step

1. **Clone the Repository**

   ```bash
   git clone https://github.com/arcosbr/pid_simulation-hydraulic_motor.git
   cd pid_simulation-hydraulic_motor
   ```

2. **Compile the Code**

   Use the following command to compile the code with GCC:

   ```bash
   gcc -o pid_sim.exe src/main.c src/pid.c src/sim.c src/gui.c src/utils.c src/log.c -lraylib -lm -lpthread -ldl -lrt -lX11
   ```

   **Notes:**
   - The linking options (`-lraylib -lm -lpthread -ldl -lrt -lX11`) may vary depending on the operating system.
   - On macOS, the command can be simplified:

     ```bash
     gcc -o pid_sim pid_sim.c -lraylib -framework OpenGL -framework Cocoa -framework IOKit
     ```

3. **Run the Simulation**

   ```bash
   ./pid_sim.exe
   ```

---

## Usage

After compiling and running the simulation, a graphical window will open displaying:

- **Real-Time Graphs**:
  - **Angle (rad)**: History of the pendulum's angle.
  - **Error (rad)**: History of the error between the setpoint and the measured angle.
  - **Pressure (bar)**: History of the pressure applied by the hydraulic motor.

- **Pendulum Visualization**:
  - Graphical representation of the controlled pendulum, updated in real-time.

- **Information and Controls**:
  - Current parameters such as simulation time, target angle, measured angle, filtered pressure, duty cycle, torques, and PID gains.

---

## Controls

Interact with the simulation using the following keys:

- **Manual Pressure Adjustment**:
  - **Up Arrow (`↑`)**: Increases manual pressure by +1 bar.
  - **Down Arrow (`↓`)**: Decreases manual pressure by -1 bar.

- **PID Gain Adjustments**:
  - **F2**: Decreases \( K_p \) by 10.
  - **F3**: Increases \( K_p \) by 10.
  - **F4**: Decreases \( K_i \) by 0.01.
  - **F5**: Increases \( K_i \) by 0.01.
  - **F6**: Decreases \( K_d \) by 10.
  - **F7**: Increases \( K_d \) by 10.

- **Integration Method**:
  - **Tab**: Toggles between Euler and Runge-Kutta 4th-order integration methods.

- **External Disturbance**:
  - **P**: Enables/Disables external disturbance.
  - **J**: Increases the magnitude of external disturbance.
  - **K**: Decreases the magnitude of external disturbance.

- **Simulation Reset**:
  - **R**: Resets the simulation to initial values.

---

## Physical Validation

The simulation strives to faithfully adhere to real-world physics through the following components:

1. **Gravity Torque**:
   - Calculated as \( T_g = -m \cdot g \cdot \frac{L}{2} \cdot \sin(\theta) \), representing the torque exerted by gravity on the pendulum's center of mass.

2. **Friction Torque**:
   - Includes viscous friction proportional to angular velocity (\( T_f = -c \cdot \omega \)).

3. **Motor Torque**:
   - Calculated from hydraulic fluid pressure using \( T_m = \frac{P \cdot D}{2 \pi} \cdot \text{scaling factor} \), suitable for hydraulic motors.

4. **Moment of Inertia**:
   - Determined as \( I = \frac{1}{3} \cdot m \cdot L^2 \), appropriate for a uniform rod pivoted at one end.

5. **Numerical Integration**:
   - Utilizes standard methods (Euler and RK4) with time subdivisions for increased stability, suitable for sensitive dynamic systems.

6. **Signal Filtering**:
   - Applies an exponential filter to smooth pressure, reducing abrupt oscillations, as practiced in real systems.

7. **Sensor Noise**:
   - Simulates sensor noise, adding realism to angle measurements.

### Note

The simulation's fidelity depends on the alignment of parameters (mass, friction, hydraulic displacement, etc.) with the real system. It is recommended to conduct experimental tests to validate and adjust parameters as necessary.

---

## License

This project is licensed under the [MIT License](LICENSE), which permits both commercial and non-commercial use. See the `LICENSE` file for more information.

---

## Contribution

Contributions are welcome! Feel free to open issues or submit pull requests to enhance this project.

---

## Acknowledgments

- **Raylib**: For providing an easy-to-use library for graphical interfaces.
- **Open Source Community**: For their continuous support and resources.
