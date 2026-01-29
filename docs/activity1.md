# ROS2 Topics (Publisher and Subscriber)

---

## 1) Overview

In this activity, you will build a simple ROS2 topic pipeline:

- **Node:** `number_publisher`  
  Publishes an `Int64` number on **`/number`**

- **Node:** `number_counter`  
  Subscribes to **`/number`**, keeps a running total, and publishes the updated value on **`/number_count`**

### Expected ROS2 graph

- **Blue boxes** → nodes  
- **Green boxes** → topics

### Expected terminal output

If you run:

```bash
ros2 topic echo /number_count
```

You should see increasing values, for example:

```text
data: 38
data: 40
data: 42
```

---

## 2) Message Type

Both nodes use the message type: **`example_interfaces/msg/Int64`**.

Inspect it with:

```bash
ros2 interface show example_interfaces/msg/Int64
```

Output:

```text
int64 data
```

This message contains:
- **Field:** `data`
- **Type:** 64-bit signed integer (`int64`)

---

## 3) Node 1 – `number_publisher` (Publisher only)

This node publishes a constant integer value every second on the topic **`/number`**.

### Python code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')

        self.get_logger().info('number_publisher started')
        self.publisher_ = self.create_publisher(Int64, '/number', 10)

        self.number_to_publish = 2
        self.create_timer(1.0, self.publish_number)

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_to_publish
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing on /number: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### What this node does

- Creates a **publisher** on `/number`
- Uses a **timer** to publish once per second
- Always publishes the value **2**

!!! note "Note"
    The queue size `10` is the QoS history depth for this topic.

---

## 4) Node 2 – `number_counter` (Subscriber + Publisher)

This node subscribes to `/number`, adds incoming values to a counter, and publishes the updated counter on `/number_count`.

### Python code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounter(Node):
    def __init__(self):
        super().__init__('number_counter')

        self.get_logger().info('number_counter started')
        self.counter = 0

        self.subscriber_ = self.create_subscription(
            Int64,
            '/number',
            self.listener_callback,
            10
        )

        self.publisher_ = self.create_publisher(Int64, '/number_count', 10)

    def listener_callback(self, msg: Int64):
        self.counter += msg.data

        out_msg = Int64()
        out_msg.data = self.counter
        self.publisher_.publish(out_msg)

        self.get_logger().info(
            f'Received: {msg.data} | Counter updated -> publishing on /number_count: {out_msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### What this node does

- Subscribes to `/number`
- Each received message is added to a **counter**
- Publishes the updated counter **inside the callback** to `/number_count`

!!! tip "Tip"
    You can verify subscriptions/publishers with:
    ```bash
    ros2 topic info /number
    ros2 topic info /number_count
    ```

---

## 5) `setup.py` Entry Points

To run the nodes using `ros2 run`, they must be registered as console scripts.

Example:

```python
entry_points={
    'console_scripts': [
        'tilin = my_first_robot_package.my_first_node:main',
        'c3po = my_first_robot_package.cosa:main',
        'number_publisher = my_first_robot_package.number_publisher:main',
        'number_counter = my_first_robot_package.number_counter:main',
    ],
},
```

!!! warning "Common mistake"
    After editing `setup.py`, you must rebuild (`colcon build`) and source the workspace again.

---

## 6) Build and Run (Step-by-step)

### Build the workspace

Run from the workspace root:

```bash
cd ~/your_ros2_workspace
colcon build --symlink-install
```

### Source ROS2 and the workspace

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```

### Run the nodes

**Terminal 1**

```bash
ros2 run my_first_robot_package number_publisher
```

**Terminal 2**

```bash
ros2 run my_first_robot_package number_counter
```

**Terminal 3**

```bash
ros2 topic echo /number_count
```

---

## 7) Troubleshooting

### StopIteration / entry point error

Make sure you sourced the workspace in **every terminal**:

```bash
source /opt/ros/<distro>/setup.bash
source ~/your_ros2_workspace/install/setup.bash
```

### Clean build (if needed)

```bash
cd ~/your_ros2_workspace
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Verify executables

```bash
ros2 pkg executables my_first_robot_package
```

Expected output:

```text
number_publisher
number_counter
```

!!! note "If you don't see them"
    Check:
    - Your file names match (`number_publisher.py`, `number_counter.py`)
    - Your `setup.py` entry points match the module names
    - You rebuilt and sourced again
