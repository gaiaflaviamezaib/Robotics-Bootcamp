## Software Architecture

The project is organized into separate folders according to function. Each folder separates a specific part of the robot system, such as hardware control, navigation logic, reusable algorithms, or bootcamp task scripts.

```text
Robotics-Bootcamp/
├── hardware/                         # Hardware interface modules
│   ├── dc_motor.py                   # DC motor control
│   ├── servo.py                      # Servo control using the PCA9685 controller
│   ├── line_sensor.py                # Infrared line sensor readings
│   ├── ultrasonic.py                 # Ultrasonic distance measurement
│   ├── oled_display.py               # OLED display support
│   └── leds.py                       # LED control
│
├── bootcamp_tasks/                   # Individual bootcamp exercise scripts
│   └── robotControlClass.py          # High-level robot control wrapper
│
├── algorithms/                       # Reusable algorithm modules
│   └── pid_controller.py             # PID controller logic
│
├── navigation/                       # Main navigation programs
│   ├── infinity_circuit.py           # PID-based line following
│   ├── simple_traversal.py           # Basic intersection traversal
│   ├── smart_manhattan.py            # Grid navigation
│   └── smart_manhattan_obstacles.py  # Grid navigation with obstacle avoidance
│
└── reset_robot.py                    # Safely stops motors and resets servos
```

---

## Smart Grid Navigation

The main navigation feature of this project is implemented in `navigation/smart_manhattan.py`.

This script allows the robot to navigate a Manhattan-style grid made of intersecting black lines. The user provides a starting coordinate and a destination coordinate. The program then calculates a route and converts that route into driving instructions that the robot can execute on the physical track.

### 1. Grid Representation

The track is represented as a coordinate system.

- `X` represents the horizontal position.
- `Y` represents the vertical position.
- Intersections are represented as nodes.
- Connections between intersections are represented as edges.

For example, a route may pass through coordinates such as:

```python
[(0, 1.0), (1, 1.0), (1, 0.0), (2, 0.0)]
```

This means that the robot should move from `(0, 1.0)` to `(1, 1.0)`, then to `(1, 0.0)`, and finally to `(2, 0.0)`.

### 2. Route Calculation

Before the robot starts moving, the program calculates a route from the starting point to the destination.

The navigation script uses a weighted shortest-path approach. Moving straight has a lower cost, turning has a higher cost, and making a U-turn has the highest cost. This is important because the robot uses Ackermann steering and cannot rotate in place like a tank-style robot.

This approach helps the robot choose a route that is not only short, but also practical for its physical steering mechanism.

### 3. Converting Coordinates into Driving Instructions

The robot does not have GPS or a global positioning sensor. Instead, it uses its current heading and the next coordinate in the calculated path.

The program compares the robot's current direction with the next required movement direction. It then converts the coordinate path into simple driving instructions:

- `straight`: continue forward through the intersection.
- `left`: turn left at the intersection.
- `right`: turn right at the intersection.
- `stop`: stop when the destination is reached.

These instructions allow the robot to follow the calculated path using only line sensors and its known starting orientation.

### 4. Intersection Detection and Turning

The robot detects intersections using its three infrared sensors: left, middle, and right.

When the sensors detect an intersection, the robot stops briefly and checks the next instruction in the route. If the next instruction is `left` or `right`, the robot performs a specific turning maneuver.

Because the robot uses Ackermann steering, it cannot spin in place. Instead, it completes turns using a controlled sequence of movements:

1. Stop at the intersection.
2. Center the wheels.
3. Steer in the required direction.
4. Move forward and backward to align the robot with the new line.
5. Use the infrared sensors to detect the new line.
6. Straighten the wheels and continue driving.

This process allows the robot to complete turns reliably despite its car-like steering system.

---

## Additional Features

### PID Line Following

The PID-based line-following system is implemented in:

```text
navigation/infinity_circuit.py
```

The robot uses a PID controller to follow the black line smoothly. PID stands for:

- Proportional
- Integral
- Derivative

The infrared sensors measure the robot's position relative to the line. These readings are converted into an error value, which represents how far the robot is from the center of the line.

The PID controller uses this error to calculate a steering correction. This correction is sent to the front-wheel servo so the robot can adjust its direction while moving forward.

This improves stability and reduces unnecessary zigzagging.

### Obstacle Avoidance

Obstacle-aware navigation is implemented in:

```text
navigation/smart_manhattan_obstacles.py
```

This script extends the grid navigation system by using the ultrasonic sensor to detect obstacles in front of the robot.

If an obstacle is detected, the program treats the blocked route as unavailable. It then recalculates a new path to the destination using the remaining available connections in the grid.

This allows the robot to adapt to changes in the environment instead of stopping permanently when a path is blocked. If necessary, the robot can choose an alternative route or perform a U-turn.

---

## Code Organization Summary

The main project files are organized as follows:

- `hardware/`: contains low-level modules for controlling motors, servos, sensors, LEDs, and the OLED display.
- `bootcamp_tasks/`: contains learning exercises and the high-level `robotControlClass.py` wrapper used by the navigation scripts.
- `algorithms/`: contains reusable algorithm logic, including the PID controller.
- `navigation/`: contains the main robot behavior scripts, including line following, grid navigation, and obstacle-aware navigation.
- `reset_robot.py`: safely stops the motors, resets the servos, and returns the robot to a safe state.
