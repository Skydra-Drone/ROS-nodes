
### Scout Drone: Remaining Nodes

The Scout Drone is the most complex, and you've already completed the hardest parts (the perception pipeline). The remaining nodes handle mission logic and communication.

#### 1. `decision_node_scout.py`
*   **Purpose:** This is the "mission brain" for the Scout. It decides what the drone should be doing at any given time (searching, loitering, returning home).
*   **Subscribes to:**
    *   `/mission/command` (from the Base Station): To receive high-level commands like "START_SEARCH", "RETURN_HOME".
    *   `/mission/target_coordinates` (from your `geolocation_node`): When a target is found, this is the trigger to change behavior (e.g., stop searching and start circling).
    *   `/mavros/state`: To know if the drone is armed and in a mode that allows autonomous control (like `GUIDED`).
*   **Publishes to:**
    *   `/mavros/setpoint_position/local` or `/mavros/setpoint_raw/global`: To send movement commands (waypoints, loiter points) to the Scout's own Flight Controller.
    *   `/mavros/cmd/set_mode`: To change the flight mode (e.g., switch to `GUIDED` mode to start the mission or `RTL` to return).
*   **Core Logic:**
    1.  Starts in an "IDLE" state.
    2.  When it receives a "START_SEARCH" command, it generates a series of GPS waypoints for a grid search pattern.
    3.  It then enters a "SEARCHING" state, sending one waypoint at a time to the Flight Controller and monitoring when it's reached.
    4.  If it receives a target coordinate while in the "SEARCHING" state, it cancels the grid search, enters a "TRACKING" or "LOITERING" state, and commands the drone to circle the target's GPS location.
    5.  If it receives a "RETURN_HOME" command, it commands the FC to enter `RTL` mode.

#### 2. `communication_node_scout.py`
*   **Purpose:** To be the gateway between the Scout's internal ROS world and the external network (Base Station). It gathers data, compresses it where necessary, and sends it out over Wi-Fi. It also listens for incoming commands.
*   **Subscribes to:**
    *   `/camera/image_raw/compressed`: To get the compressed video stream. (You'll need to run a separate `image_transport` republish node to create this topic from `/camera/image_raw`).
    *   `/yolo/detections`: To send alerts to the base station when something is detected.
    *   `/mission/target_coordinates`: To send the final target GPS to the base.
    *   All necessary telemetry topics from MAVROS (e.g., `/mavros/global_position/global`, `/mavros/battery`).
*   **Publishes to:**
    *   `/mission/command` (internal ROS topic): When it receives a command packet from the Base Station's network socket, it publishes it here for the `decision_node_scout` to hear.
*   **Core Logic:**
    *   Uses standard Python networking libraries (`socket`, `zeromq`, etc.) to create TCP or UDP clients/servers.
    *   In a loop, it takes the latest data from its subscribed topics, serializes it (e.g., into a JSON string), and sends it over the network to the Base Station's IP address.
    *   It also runs a listener in a separate thread to receive command packets from the Base Station.

#### 3. `delivery_commander_node.py`
*   **Purpose:** This is the special node that acts as the "brain" for the Delivery Drone. It translates the target GPS coordinate into direct MAVLink commands.
*   **Subscribes to:**
    *   `/mission/target_coordinates`: Gets the final target location from the `geolocation_node`.
    *   `/mission/command`: Listens for the "APPROVE_DELIVERY" command from the Base Station.
*   **Publishes to:**
    *   **Nowhere in ROS.** This node's output is not a ROS topic. Its output is MAVLink packets sent directly over a network socket to the Delivery Drone's IP address.
*   **Core Logic:**
    1.  Waits in an "IDLE" state.
    2.  When it receives a target coordinate, it stores it and enters a "PENDING_APPROVAL" state.
    3.  When it receives the "APPROVE_DELIVERY" command, it enters the "COMMANDING" state.
    4.  It then uses a library like `pymavlink` to manually construct MAVLink messages (e.g., `MAV_CMD_NAV_WAYPOINT`, `MAV_CMD_DO_SET_SERVO`).
    5.  It sends these MAVLink packets via a UDP socket to the Delivery Drone's Flight Controller.
    6.  After sending the sequence of commands (fly to target, release payload, return home), it goes back to "IDLE".

---

### Base Station: Remaining Nodes

The Base Station nodes are primarily for user interface and relaying information. They run on your laptop.

#### 1. `communication_node_base.py`
*   **Purpose:** The mirror image of the Scout's communication node. It receives network data from the drone(s) and publishes it as ROS topics for the GUI nodes to use. It also sends operator commands back to the drone.
*   **Publishes to (internal Base Station ROS):**
    *   `/scout/camera/image_raw/compressed`
    *   `/scout/yolo/detections`
    *   `/scout/mission/target_coordinates`
    *   `/scout/telemetry/gps`, `/scout/telemetry/battery`, etc.
    *   `/delivery/telemetry/gps` (receives telemetry from the Delivery Drone's FC).
*   **Subscribes to (internal Base Station ROS):**
    *   `/ui/mission_command`: A topic that the GUI publishes to when the operator clicks a button.
*   **Core Logic:**
    *   Runs a network server to listen for incoming data from the drones.
    *   When a data packet arrives, it deserializes it and publishes the contents to the appropriate ROS topic.
    *   When it receives a message on `/ui/mission_command`, it serializes it and sends it over the network to the Scout Drone.

#### 2. GUI / Operator Interface Nodes
*   **Purpose:** To provide a user interface for the operator. This is often not a single node but a combination of standard ROS tools and potentially a custom GUI.
*   **Tools to Use:**
    *   **`rqt_image_view`**: A simple tool to display the `/scout/camera/image_raw/compressed` video stream.
    *   **RViz**: A powerful 3D visualization tool. You can configure it to show:
        *   The drone's position (`/scout/telemetry/gps`) on a satellite map background (using `rviz-satellite` plugin).
        *   The target's position (`/scout/mission/target_coordinates`) as a marker on the map.
    *   **Custom GUI (e.g., using `PyQt` or `rqt_gui` plugin):** A simple window with buttons ("Start Search", "Approve Delivery", "Return Home"). When a button is clicked, it publishes a message to the `/ui/mission_command` topic.

---

### Development Chronology

Your plan should be:

1.  **Done:** `camera_node`, `yolo_node`, `geolocation_node` (The Perception Pipeline).
2.  **Next:** `communication_node_scout` and `communication_node_base`. Get basic telemetry (like GPS) sending from the Scout and displaying on the Base. This is a huge step.
3.  **Then:** Implement the GUI nodes on the Base Station. Create simple buttons that publish commands.
4.  **After:** Implement the `decision_node_scout` to listen for these commands and make the simulated drone fly simple patterns.
5.  **Finally:** Implement the `delivery_commander_node` to send the final MAVLink commands. This can be tested by sending commands to a second instance of the SITL simulator representing the delivery drone.