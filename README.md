# 🤖 Robotics-Bootcamp : Autonomous Line-Following Robot

Hey there! 👋 Welcome to my autonomous robot project.

This repository holds all the code for a **Raspberry Pi**-powered robot I built. The main goal? Getting it to navigate a city-like grid on its own, avoiding obstacles, and smoothly following lines like a pro.

Unlike basic robots that turn like tanks (by spinning their wheels in opposite directions), this one uses **Ackermann steering**—just like a real car. This makes coding its movements a lot more fun (and challenging)!

---

## 🛠 What's Under the Hood? (Hardware)

Here is what makes the robot tick:

| Component | What it does |
|---|---|
| **Raspberry Pi** | The main brain of the operation (GPIO, I2C, PWM) |
| **2× DC Motors** | Pushes the robot forward (rear-wheel drive) |
| **3× Servomotors** | Turns the front wheels and rotates the "head" |
| **Adafruit PCA9685** | A 16-channel controller to handle all the servos smoothly |
| **3× IR Sensors** | Looks at the floor to track the black line (left, middle, right) |
| **Ultrasonic Sensor** | The robot's eyes to spot obstacles blocking the road |
| **OLED Display** | A tiny screen to show what the robot is thinking |

---

## 🗺️ Smart Grid Navigation (The Cool Part)

The biggest achievement in this project is the **Smart Manhattan** navigation script. Imagine dropping the robot in a mini-city of crossing lines. You tell it "go from intersection A to intersection B", and it figures out the rest.

### 1. The Map
The track is mapped out like an X/Y graph:
* **X** is horizontal (from 0 to 3).
* **Y** is vertical.
The intersections are treated as "nodes" that the robot travels between.

### 2. Finding the Best Route
Before moving an inch, the robot runs a **BFS (Breadth-First Search)** algorithm in its head to find the absolute shortest path to the destination.
It outputs a list of coordinates, like this: `[(0, 1.0), (1, 1.0), (1, 0.0), (2, 0.0)]`

### 3. Knowing Where to Turn
Because the robot doesn't have a real GPS, it uses math (Dot and Cross products) based on its current heading to translate those coordinates into real driving instructions:
* `straight` (Keep going!)
* `left` (Turn left here)
* `right` (Turn right here)

### 4. Making the Turn
Since our robot steers like a car, it can't just spin on the spot at an intersection. When the sensors detect a crossroad, the robot does a specific parking maneuver: it rolls forward a bit, cranks the wheels, reverses to align itself, and then centers back on the new line. 

---

## 🚀 Other Cool Features

### Smooth Driving (PID Controller)
**Check it out here:** `navigation/infinity_circuit.py`

To stop the robot from zigzagging crazily over the line, it uses a **PID** (Proportional, Integral, Derivative) algorithm. This does some quick math to adjust the steering wheel perfectly so it glides along the track.

### Dynamic Obstacle Avoidance
**Check it out here:** `navigation/smart_manhattan_obstacles.py`

What if someone drops a box on the road? The robot's ultrasonic sensor is always scanning ahead. If a street is blocked, the robot instantly crosses that street off its mental map, calculates a brand new route, and does a U-turn if it has to.

---

## 📂 How the Code is Organized

```text
Robotics-Bootcamp/
├── hardware/               # How to talk to the motors and sensors
├── bootcamp_tasks/         # Smaller test scripts from learning the basics
├── algorithms/             # The heavy math stuff (like the PID logic)
├── navigation/             # The main event:
│   ├── infinity_circuit.py # Endless track follower
│   ├── simple_traversal.py # Basic intersection driving
│   ├── smart_manhattan.py  # The smart grid GPS
│   └── smart_manhattan_obstacles.py # The grid GPS + Obstacle avoidance
└── reset_robot.py          # A panic button script to stop the motors!
```

---

## ⚙️ Want to try it out?

1. Install the required Python packages on your Raspberry Pi:
```bash
pip install RPi.GPIO adafruit-circuitpython-pca9685 luma.oled
```

2. Launch the smart grid navigator:
```bash
python3 navigation/smart_manhattan.py
```
*(It will pop up a map in the terminal and ask you where you want to go!)*
