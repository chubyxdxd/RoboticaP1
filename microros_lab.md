# micro-ROS Laboratory Guide

## Part 1: Theory

### What is micro-ROS?

micro-ROS is a robotic framework that brings ROS 2 (Robot Operating System 2) to microcontrollers and embedded systems. It enables resource-constrained devices to seamlessly integrate into the ROS 2 ecosystem, allowing direct communication with ROS 2 nodes running on more powerful computers.

### Key Concepts

#### Architecture
- **micro-ROS Client Library**: A highly optimized version of ROS 2's client library (rclc) designed for microcontrollers
- **micro-ROS Agent**: Acts as a bridge between micro-ROS nodes and the ROS 2 network
- **Communication Protocol**: Uses Micro XRCE-DDS (eXtremely Resource Constrained Environments DDS) for efficient communication

#### Benefits
- **Resource Efficiency**: Minimal memory footprint suitable for MCUs with limited RAM and Flash
- **Real-time Capable**: Supports deterministic behavior for time-critical applications
- **ROS 2 Compatible**: Full integration with existing ROS 2 tools and workflows
- **Multi-platform**: Supports various microcontroller platforms (ESP32, STM32, Teensy, etc.)

#### Communication Model
- **Publishers**: Send data to topics
- **Subscribers**: Receive data from topics
- **Services**: Request-response pattern
- **Topics**: Named channels for message exchange

### Hardware Requirements
- Microcontroller board (ESP32, Arduino, Teensy, etc.)
- USB connection for programming and communication
- Optional: Input devices (buttons, sensors) and output devices (LEDs, motors)

### Software Requirements
- micro-ROS build system
- ROS 2 installation (Humble, Iron, or newer)
- micro-ROS Agent
- Platform-specific toolchain (Arduino IDE, PlatformIO, etc.)

---

## Part 2: Example - Robot Control with Buttons

### Overview
This example demonstrates creating a micro-ROS node that:
- Publishes velocity commands to `/cmd_vel` topic using 4 buttons (forward, backward, left, right)
- Subscribes to a topic to receive feedback or commands

### Hardware Setup
- 4 push buttons connected to GPIO pins
- Microcontroller (ESP32 example)
- Connection diagram:
  - Button 1 (Forward): GPIO 25
  - Button 2 (Backward): GPIO 26
  - Button 3 (Left): GPIO 27
  - Button 4 (Right): GPIO 14
  - All buttons connected with pull-down resistors or internal pull-down enabled

### Software Implementation

#### Step 1: Include Required Libraries

```cpp
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// Pin definitions
#define BTN_FORWARD 25
#define BTN_BACKWARD 26
#define BTN_LEFT 27
#define BTN_RIGHT 14

// Robot parameters
#define LINEAR_SPEED 0.5   // m/s
#define ANGULAR_SPEED 1.0  // rad/s
```

#### Step 2: Initialize Variables

```cpp
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__String feedback_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}
```

#### Step 3: Subscriber Callback

```cpp
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  // Process received message
  Serial.print("Received: ");
  Serial.println(msg->data.data);
}
```

#### Step 4: Timer Callback for Publishing

```cpp
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read button states
    bool forward = digitalRead(BTN_FORWARD);
    bool backward = digitalRead(BTN_BACKWARD);
    bool left = digitalRead(BTN_LEFT);
    bool right = digitalRead(BTN_RIGHT);
    
    // Reset velocities
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    
    // Set linear velocity based on forward/backward buttons
    if (forward) {
      twist_msg.linear.x = LINEAR_SPEED;
    } else if (backward) {
      twist_msg.linear.x = -LINEAR_SPEED;
    }
    
    // Set angular velocity based on left/right buttons
    if (left) {
      twist_msg.angular.z = ANGULAR_SPEED;
    } else if (right) {
      twist_msg.angular.z = -ANGULAR_SPEED;
    }
    
    // Publish the message
    RCSOFTCHECK(rcl_publish(&publisher, &twist_msg, NULL));
  }
}
```

#### Step 5: Setup Function

```cpp
void setup() {
  Serial.begin(115200);
  
  // Configure button pins
  pinMode(BTN_FORWARD, INPUT);
  pinMode(BTN_BACKWARD, INPUT);
  pinMode(BTN_LEFT, INPUT);
  pinMode(BTN_RIGHT, INPUT);
  
  // Configure micro-ROS transport
  set_microros_transports();
  
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "microros_button_controller", "", &support));
  
  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "robot_feedback"));
  
  // Create timer (100ms = 10Hz)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &feedback_msg, 
    &subscription_callback, ON_NEW_DATA));
  
  // Allocate memory for subscriber message
  feedback_msg.data.data = (char *) malloc(100 * sizeof(char));
  feedback_msg.data.size = 0;
  feedback_msg.data.capacity = 100;
  
  Serial.println("micro-ROS node initialized!");
}
```

#### Step 6: Loop Function

```cpp
void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
```

### Running the Example

#### On the microcontroller side:
1. Upload the code to your microcontroller
2. Connect via USB

#### On the computer side:
1. Start the micro-ROS agent:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

2. Verify the node is running:
```bash
ros2 node list
ros2 topic list
```

3. Echo the velocity commands:
```bash
ros2 topic echo /cmd_vel
```

4. (Optional) Publish to the feedback topic:
```bash
ros2 topic pub /robot_feedback std_msgs/msg/String "data: 'Robot responding!'"
```

### Expected Behavior
- Pressing **Forward**: Publishes positive linear.x velocity
- Pressing **Backward**: Publishes negative linear.x velocity
- Pressing **Left**: Publishes positive angular.z velocity (counterclockwise)
- Pressing **Right**: Publishes negative angular.z velocity (clockwise)
- The subscriber receives messages on `/robot_feedback` topic

---

## Part 3: Challenges

### Challenge 1: Emergency Stop Button
**Difficulty**: Easy

Add a fifth button that acts as an emergency stop. When pressed, it should:
- Immediately publish zero velocities to `/cmd_vel`
- Disable all other buttons until the emergency stop is released
- Publish a warning message to a new topic `/emergency_status`
- This new topic should work inside Potential field algorithm. 

**Hints**:
- Use a boolean flag to track emergency state
- Check this flag before processing other buttons
- Create an additional publisher for the emergency status

---

### Challenge 2: Speed Control with Potentiometer
**Difficulty**: Medium

Replace the fixed speed values with variable speeds controlled by a potentiometer:
- Read analog input from the potentiometer
- Map the value to a speed range (0.0 to 1.0 m/s for linear, 0.0 to 2.0 rad/s for angular)
- Display the current speed setting via a new publisher on `/speed_status`

**Hints**:
- Use `analogRead()` to read the potentiometer
- Map values using: `map()` or manual calculation
- Update speed values before setting twist message

---

### Challenge 3: Button Combination Commands
**Difficulty**: Medium

Implement diagonal movement by detecting button combinations:
- Forward + Left = Move forward-left
- Forward + Right = Move forward-right
- Backward + Left = Move backward-left
- Backward + Right = Move backward-right

**Requirements**:
- Both linear and angular velocities should be active simultaneously
- Scale velocities appropriately (e.g., 70% of max speed for smoother diagonal movement)

**Hints**:
- Check multiple button states together
- Set both `linear.x` and `angular.z` in the same message
- Consider using reduced speeds: `LINEAR_SPEED * 0.7`

---

### Challenge 4: Odometry Subscriber and Display
**Difficulty**: Medium-Hard

Subscribe to the `/odom` topic and display robot position on an OLED/LCD screen:
- Subscribe to `nav_msgs/msg/Odometry`
- Extract position (x, y) and orientation (yaw)
- Display this information on a connected display
- Update display at 1Hz

**Hints**:
- You'll need to add the `nav_msgs` message type
- Convert quaternion to Euler angles for yaw
- Use a library like Adafruit_SSD1306 for OLED displays

---

### Challenge 5: Multi-Robot Controller
**Difficulty**: Hard

Modify the code to control multiple robots with namespace support:
- Add a toggle button to switch between robots
- Use namespaces for topics: `/robot1/cmd_vel`, `/robot2/cmd_vel`
- Display which robot is currently being controlled (LED or serial)
- Each robot should have independent velocity commands

**Requirements**:
- Create publishers for both robot namespaces
- Implement smooth switching between robots
- Add visual feedback for the active robot

**Hints**:
- Create multiple publishers during setup
- Use a state variable to track active robot
- Publish to the appropriate publisher based on state

---

### Challenge 6: Gesture-Based Control
**Difficulty**: Hard

Replace buttons with an accelerometer for gesture-based control:
- Tilt forward/backward controls linear velocity
- Tilt left/right controls angular velocity
- Implement a "dead zone" to prevent accidental movements
- Add shake detection to stop the robot

**Requirements**:
- Integrate an IMU sensor (MPU6050, LSM9DS1, etc.)
- Read accelerometer data
- Map tilt angles to velocities
- Implement filtering to smooth out noise

**Hints**:
- Use libraries like `Adafruit_MPU6050`
- Calculate tilt using `atan2()`
- Apply exponential moving average for smoothing
- Define threshold values for dead zones

---

### Bonus Challenge: Complete Teleoperation System
**Difficulty**: Expert

Create a complete bidirectional teleoperation system:
- Control robot with buttons (as in example)
- Subscribe to `/odom`, `/scan`, and `/battery_state`
- Display robot status on OLED/LCD:
  - Position and heading
  - Battery level
  - Closest obstacle distance
- Add haptic feedback (vibration motor) when obstacles are near
- Implement auto-stop if battery is critically low
- Add data logging to SD card

**Requirements**:
- Multiple subscribers (odometry, laser scan, battery)
- Status display with real-time updates
- Safety features (auto-stop, warnings)
- Persistent logging

---

## Additional Resources

### Useful ROS 2 Commands
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Get topic info
ros2 topic info /cmd_vel

# Echo topic messages
ros2 topic echo /cmd_vel

# Check message type
ros2 interface show geometry_msgs/msg/Twist

# Start micro-ROS agent (serial)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Start micro-ROS agent (UDP)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Debugging Tips
1. **Connection Issues**: Verify the correct serial port and baud rate
2. **Node Not Appearing**: Check if micro-ROS agent is running
3. **Messages Not Publishing**: Add serial debug prints to verify code execution
4. **Memory Issues**: Reduce message frequency or buffer sizes
5. **Timing Problems**: Adjust executor spin time and timer intervals

### Further Learning
- [micro-ROS Official Documentation](https://micro.ros.org/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [micro-ROS Tutorials](https://micro.ros.org/docs/tutorials/)
- [RCLC API Documentation](https://micro.ros.org/docs/api/rclc/)

---

## Submission Guidelines

For each challenge you complete:
1. Document your approach and implementation
2. Include code with comments
3. Add photos/videos of the working system
4. Describe any problems encountered and solutions
5. Suggest improvements or extensions

Good luck with your micro-ROS experiments!
