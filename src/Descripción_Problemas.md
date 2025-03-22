# Problem Description

This document explains how I solved the problems given in the exam. Here, I detail step by step what I did in the programs `problema1.py` and `problema2.py`, explaining the decisions I made and how I arrived at the solution. I hope it's clear and easy to understand.

---

## Problem 1: Managing Turtles in Turtlesim

### **What does this program do?**
The program `problema1.py` interacts with the `turtlesim` simulator to perform two main tasks:
1. Delete the default turtle that appears in the simulator (`turtle1`).
2. Create several turtles in random positions within the simulator's area.

### **How did I solve it?**
1. **First, I initialized the ROS node**:
   - I used the `rospy` library to create a node called `nodo_tortugas`. This is mandatory for the program to communicate with ROS.
   - Basically, the node is like the "brain" of the program that controls everything.

2. **Then, I deleted the initial turtle**:
   - I used the `/kill` service provided by `turtlesim`. This service allows deleting turtles by their name.
   - I created a method called `borrar_tortuga_inicial` that waits for the service to be available and then calls it to delete the turtle named `turtle1`.

3. **Next, I generated turtles in random positions**:
   - I used the `/spawn` service to create new turtles.
   - In the method `crear_tortugas_random`, I generated random coordinates for the turtles' positions using the `random` library.
   - I gave each turtle a unique name like `tortuga1`, `tortuga2`, etc., to avoid conflicts.

4. **Finally, I put everything together in the main block**:
   - I created an instance of the `Tortugas` class.
   - I called the method to delete the initial turtle and then the method to create 5 new turtles.

### **Why did I do it this way?**
- I used a class (`Tortugas`) to better organize the code and make it easier to understand.
- I added log messages (`rospy.loginfo`, `rospy.logwarn`, etc.) so the program could tell me what was happening at every moment.

---

## Problem 2: Controlling Turtle Movement

### **What does this program do?**
The program `problema2.py` controls the movement of the turtles created in the `turtlesim` simulator. The idea is for the turtles to move according to certain rules I defined.

### **How did I solve it?**
1. **First, I initialized the ROS node**:
   - Just like in Problem 1, I created a node called `nodo_movimiento` to handle everything related to the turtles' movement.

2. **Then, I subscribed to the position topics**:
   - I used the topic `/turtleX/pose` (where `X` is the turtle's number) to get the current position of each turtle.
   - This allowed me to know where each turtle was at all times.

3. **Next, I published velocity commands**:
   - I used the topic `/turtleX/cmd_vel` to send linear and angular velocity commands to the turtles.
   - Basically, this is what makes the turtles move.

4. **I implemented the movement logic**:
   - I created a loop that calculates where the turtles should move and at what speed.
   - I also added conditions to prevent the turtles from leaving the simulator's area.

### **What logic did I follow?**
- I used subscribers and publishers because it's the standard way to interact with topics in ROS.
- I added log messages to know what each turtle was doing at every moment.

---

## How to Run the Programs

### **Prerequisites**
1. Have ROS and the `turtlesim` package installed.
2. Make sure the scripts `problema1.py` and `problema2.py` are executable. To do this, use this command in the terminal:
   ```bash
   chmod +x problema1.py problema2.py

## Launching the Programs
I created a .launch file to run everything at once. This file launches the turtlesim node and both programs (problema1.py and problema2.py). Here's what the file looks like:

<launch>
    <!-- Launch the turtlesim_node from the turtlesim package -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>

    <!-- Launch the script problema1.py -->
    <node pkg="First-Exam-LRT4102" type="problema1" name="turtle_node" output="screen"/>

    <!-- Launch the script problema2.py -->
    <node pkg="First-Exam-LRT4102" type="problema2" name="movement_node" output="screen"/>
</launch>

To run it, simply use this command:

```python
roslaunch First-Exam-LRT4102 launch_file_name.launch
```

---

---

## Conclusion

Through this exam, I was able to successfully solve the proposed problems using ROS and the `turtlesim` simulator. In `problema1.py`, I managed to interact with ROS services to delete and create turtles dynamically, ensuring the program worked as expected. In `problema2.py`, I implemented logic to control the movement of turtles using ROS topics, which allowed me to apply concepts like subscribing to position data and publishing velocity commands.

Overall, the results demonstrate a solid understanding of ROS fundamentals, including the use of services, topics, and `.launch` files to manage and automate tasks. This experience helped me reinforce my knowledge and improve my problem-solving skills in robotics programming.