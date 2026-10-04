# Five-node AMR warehouse simulation

This is a self-contained workspace. It includes all source packages from the
original multi-modal warehouse workspace plus `multi_amr_coordination`.

Run the five robots in Gazebo:

```bash
cd "/home/ksp/Documents/ChatGPT/ros2 amr/multi_amr_ws"
./run_five_amr.sh
```

The five robot nodes are `r1`, `r2`, `r3`, `r4`, and `r5`. Each publishes:

```text
/<robot>/odom
/<robot>/scan
/<robot>/state
/<robot>/gnn_action
/<robot>/cmd_vel
/<robot>/message
```

To send a direct ROS message from R1, use another terminal in the same Docker
container or any device on `ROS_DOMAIN_ID=42`:

```bash
ros2 topic pub --once /r1/message std_msgs/msg/String "{data: 'R1: aisle is clear'}"
```

R2, R3, R4, and R5 log the message in their `peer_message_node` output. To
send from another robot, replace `r1` with `r2`, `r3`, `r4`, or `r5`.
