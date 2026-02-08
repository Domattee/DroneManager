Holodeck
#############

Overview
***********

Holodeck is an example for the integration of DroneManager with modern simulation environments like Unity. 
It allows Users to control a real drone with a Playstation Controller via DroneManager while experiencing the virtual 
flight in a Unity Environment with enhanced perception through VR-Glasses (e.g. Meta-Quest Pro).
This Docs Section explains how it works and how you can easily modify it or expand it for your UseCase. You can download 
this project on `Github`_.

.. _Github: https://github.com/bakethi/Holodeck

.. image:: imgs/Holodeck-SimSetup.png
   :alt: Holodeck with DroneManager and Gazebo for a full Simulation Setup in an Example Scene. The Unity Environment is running in the background and receives the drone data from DroneManager (bottom right) which is controlling a drone in Gazebo (left)
   :align: center

System Architecture
===================
The Holodeck system is built on a **Decoupled Architecture**. This means the flight logic (the "Brain") runs in a Python environment, while the simulation and visualization (the "Body") run in Unity. They communicate via a low-latency UDP network protocol.

.. image:: imgs/Holodeck-Systemdesign.svg
   :alt: Holodeck System Overview Diagram
   :align: center

Data Flow & Lifecycle
---------------------

The integration follows a continuous loop to ensure the Digital Twin stays synchronized with the real-world drone state:

1.  **Backend (Python/DroneManager):**

    * The ``External Plugin`` allows to expose certain data for other applications by packaging drone telemetry, mission stages etc. into a JSON dictionary.
    * The ``OptiTrack Plugin`` can grab the tracked position of drones of an OptiTrack System and update the drones telemetry accordingly.
    * The ``Stream Plugin`` picks up the vitual camera stream from unity and renders it in an external window via open-cv.
    * The ``Controller Plugin`` allows to control a real or simulated drone via a commercial PlayStation Controller.

2.  **Transport (UDP):** 

The backend streams this JSON via UDP to Unity. This is "fire-and-forget," ensuring the drone's flight is never delayed by rendering frames in Unity.
On the otherhand Unity is able to transmit the stream of a virtual camera via TCP and JPEG images back to DroneManager for external display via Open-CV.

3.  **Frontend (Unity/Holodeck):**

    * The ``UDPReceiver`` catches the packet and parses it according to the DroneDataClasses struct.
    * The ``DroneManager`` (Unity Side) checks if the drone exists in the scene. If not, it spawns a new one.
    * The ``DroneController`` updates the 3D model's position and orientation.
    * The ``Displays`` update AR elements (Fences/Waypoints) based on the latest mission data.
    * The ``CameraSpringArm`` attaches to the first drone and hosts the VR-Camera.
4.  **Feedback (Optional):** 

The ``CameraStreamer`` can send the virtual FPV view back to Python for Computer Vision analysis.

Key Technologies
----------------

* **Unity 3D:** Used for simulation and VR rendering.
* **JSON (Newtonsoft.Json):** The standard data format used to bridge the gap between Python and C#.
* **Asynchronous Threading:** Used in Unity to handle network traffic without causing "lag" or frame drops in the VR headset.
* **VR (XR Interaction Toolkit):** Provides the immersive interface for the pilot via the OVR Camera Rig.


Scripts
-------------------------

DroneManager (Unity Side)
^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``DroneManager.cs`` is the central orchestrator within Unity. It acts as a bridge between the raw data received via UDP and the physical GameObjects in the scene.

**Key Responsibilities:**

* **Lifecycle Management:** Automatically spawns drone prefabs when a new ID is detected in the data stream and destroys them if the connection is lost (stale data).
* **State Synchronization:** Maps incoming telemetry (position, rotation) from ``DroneDataClasses`` to the active ``DroneController`` scripts.
* **Camera Attachment:** On initialization, it automatically links the ``CameraSpringArm`` (VR view) to the first spawned drone to ensure the pilot's perspective is correctly positioned.
* **Visualizer Updates:** Coordinates the ``FenceDisplay`` and ``TargetDisplay`` components, translating coordinate lists from DroneManager into visual boundaries and waypoints in the Unity world.

.. note::
   This script must be attached to a persistent Manager object in your Unity scene and requires a reference to a Drone Prefab and the ``CameraSpringArm``.

DroneController
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``DroneController`` script is attached to the individual Drone Prefabs. Its primary purpose is to represent the physical drone within the Unity environment and ensure fluid visual movement.

**Key Responsibilities:**

* **Data Container:** Acts as the local storage for the most recent ``DroneData`` (telemetry, flight modes, and status) received from DroneManager.
* **Coordinate Transformation:** Handles the conversion from the Python-based coordinate system (e.g., MAVLink or custom ENU) to the Unity **Y-up** coordinate system. 
* **Movement Smoothing:** Instead of "snapping" to new coordinates, it uses linear interpolation (``Lerp``) and spherical interpolation (``Slerp``) to create smooth, high-frame-rate movement even if the incoming data rate is low.
* **Orientation Mapping:** Translates the Euler angles (Attitude) from the data stream into Unity Quaternions to accurately reflect the drone's pitch, roll, and yaw.

.. tip::
   You can adjust the ``positionSmoothing`` and ``rotationSmoothing`` variables in the Unity Inspector to balance between "real-time accuracy" and "visual smoothness."


DroneDataClasses
^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``DroneDataClasses`` file defines the data structures used for JSON deserialization. These classes act as "blueprints" that allow the **Newtonsoft.Json** library to map incoming UDP packets directly into C# objects.

**Key Components:**

* **RootData:** The top-level container. It uses a ``Dictionary<string, DroneData>`` to handle dynamic drone names (e.g., "Drone_01", "Alpha"), allowing the system to scale to multiple drones without hardcoded variables.
* **DroneData:** Contains the per-drone telemetry and state, including:
    * **Spatial Data:** Position (XYZ), Attitude (Roll/Pitch/Yaw), and Heading.
    * **Status Flags:** Connectivity, Arming status, and In-air detection.
    * **Augmented Reality Data:** Contains the ``fence`` coordinates and ``target`` waypoints used by the visualizer scripts.
* **MissionData:** Holds global mission parameters such as the defined flight area, the current mission stage, and battery levels for all active units.

.. important::
   These classes are marked with the ``[Serializable]`` attribute, ensuring they are compatible with the Unity Inspector for debugging and can be easily converted back to JSON if telemetry logging is required.


UDPReceiver
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``UDPReceiver`` is the networking gateway for the project. It handles asynchronous communication with the DroneManager Python backend, ensuring that high-frequency telemetry data does not block the main Unity rendering thread.

**Key Responsibilities:**

* **Multithreaded Listening:** Operates a background ``Thread`` to listen for incoming UDP packets. This prevents "stuttering" in Unity by decoupling network I/O from the frame rate.
* **Handshake Mechanism:** On startup (and periodically via a timer), it sends a ``ServerConfig`` JSON payload to the Python server's IP/Port to request data at a specific ``frequency`` and ``duration``.
* **Asynchronous Deserialization:** Converts raw byte streams into ``RootData`` objects using **Newtonsoft.Json**.
* **Global Data Access:** Provides a ``public static`` reference (``currentData``) that other scripts like ``DroneManager`` can access without needing a direct reference to the receiver object.
* **History Buffering:** Maintains a ``Queue`` of recent packets (``dataHistory``), allowing for potential future features like lag compensation or trajectory trail rendering.

.. warning::
   UDP is a connectionless protocol. If you aren't seeing any drones in the scene, verify that the ``serverIp`` matches your backend host and that the ``receivePort`` is not being blocked by a firewall.


CameraSpringArm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``CameraSpringArm`` script manages the virtual camera's behavior. It mimics a mechanical spring-arm or "boom" mount, ensuring that the First-Person View (FPV) or third-person chase view remains stable and fluid, which is critical for preventing motion sickness in VR environments.

**Key Responsibilities:**

* **Dynamic Target Acquisition:** The ``target`` transform is assigned dynamically by the ``DroneManager`` at runtime. This allows the camera to automatically "snap" to the correct drone as soon as it spawns.
* **Rotational Offset Logic:** Unlike a simple parent-child relationship, this script calculates a ``desiredPosition`` based on the target's rotation. This means the camera follows the drone's heading (yaw) and attitude, maintaining the relative ``offset`` (e.g., staying exactly 8 meters behind the drone even as it turns).
* **LateUpdate Execution:** The camera logic runs in ``LateUpdate()``. This ensures the camera only calculates its new position *after* the ``DroneController`` has finished its own movement update, eliminating "jitter" or stuttering.
* **Smooth Follow & Look-At:** * Uses **Lerp** to smoothly transition the camera position.
    * Uses **Slerp** combined with ``LookRotation`` to ensure the camera always points directly at the drone, creating a professional "tracking shot" feel.

.. tip::
   To create a "Chase Cam," set the offset to something like ``(0, 2, -5)``. To create a "Cockpit View," set the offset to ``(0, 0, 0.5)`` and increase the ``positionSmoothSpeed`` to a much higher value.

CameraStreamer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``CameraStreamer`` script enables real-time video streaming from any Unity camera to an external network client via TCP. This is particularly useful for integrating Unity with external Computer Vision (CV) pipelines or remote monitoring stations. It can be picked up by the Stream Plugin in DroneManager and displayed.

**Key Responsibilities:**

* **TCP Server Hosting:** Initializes a background ``TcpListener`` on a dedicated thread, allowing external applications to connect and receive the video stream as a series of JPEG-encoded frames.
* **Virtual Render Target:** Uses a ``RenderTexture`` as a "Virtual Film" to capture camera data without affecting what the user sees on their main display.
* **Snapshot Redirection:** Employs a "Render Swap" technique—briefly hijacking the camera's target, forcing a manual ``Render()`` call, and then restoring the original display settings.
* **Performance Control:** Includes a ``frameRate`` throttle and a ``quality`` compressor (JPEG) to minimize the impact on Unity's main thread and reduce network bandwidth consumption.
* **GPU-to-CPU Transfer:** Manages the high-performance transfer of pixel data from the GPU's ``RenderTexture`` to a CPU-accessible ``Texture2D`` for network transmission.

.. note::
   The stream sends a **Length Prefix** (4-byte integer) before each JPEG frame. Your receiving client must read these 4 bytes first to know how much memory to allocate for the incoming image.

.. tip::
   Streaming at high resolutions (e.g., 1080p) or high frame rates can significantly impact performance. For most AI/CV applications, a resolution of ``640x480`` at ``15fps`` provides an ideal balance.

FenceDisplay
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``FenceDisplay`` script provides a real-time 3D visualization of the drone's geofence boundaries. It translates the numerical safety limits from the Python backend into a semi-transparent volume within the Unity world, allowing pilots to see their "safe flight volume" in VR.

**Key Responsibilities:**

* **Coordinate Decoupling:** On startup, the script detaches the ``fenceCube`` from the drone's hierarchy (``SetParent(null)``). This ensures that while the drone moves, the geofence remains fixed in world coordinates.
* **Manual Lifecycle Management:** Because the fence is detached from the drone, the script explicitly handles the destruction of the fence object via ``OnDestroy()`` when the drone disconnects.
* **NED to Unity Conversion:** Automatically converts the standard **North-East-Down (NED)** fence format into Unity's **Left-Handed Y-up** coordinate system:
    * **North (N):** Maps to Unity **Z**.
    * **East (E):** Maps to Unity **X**.
    * **Down (D):** Maps to Unity **-Y**.
* **Dynamic Scaling:** Calculates the center position and the absolute scale (X, Y, Z) of the fence volume based on the min/max coordinate pairs provided in the data stream.
* **Visibility Toggling:** Automatically hides the visualizer (``SetActive(false)``) if no fence data is present or if the data packet is malformed.

.. note::
   The script expects a ``fenceData`` list of at least 6 floats: ``[N_Min, N_Max, E_Min, E_Max, D_Min, D_Max]``.

.. tip::
   To make the fence look like a "hologram," use a Cube with a **Transparent** or **Unlit** shader and set the Alpha value to roughly 0.3. Ensure the BoxCollider is disabled if you don't want the drone to physically bounce off the visual fence.


TargetDisplay
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``TargetDisplay`` script visualizes the drone's current navigation objective or waypoint. Like the ``FenceDisplay``, it serves as an AR overlay to help the pilot understand where the autonomous system is currently trying to go.

**Key Responsibilities:**

* **World-Space Persistence:** Detaches the ``targetSphere`` from the drone's hierarchy on ``Start()``. This ensures the waypoint stays at its specific GPS/Local coordinates while the drone moves toward it.
* **Navigation Line Rendering:** Manages an optional ``LineRenderer`` component that draws a physical line between the drone and its target, providing a clear visual representation of the current flight path.
* **Coordinate Mapping:** Converts **North-East-Down (NED)** waypoint coordinates into the Unity coordinate system:
    * **North** $\rightarrow$ Unity **Z**
    * **East** $\rightarrow$ Unity **X**
    * **Down** $\rightarrow$ Unity **-Y**
* **Dynamic Visibility:** Automatically disables both the target mesh and the path line if the ``targetData`` becomes null or contains insufficient data, cleaning up the HUD when no active mission is running.
* **Clean-up Logic:** Ensures the detached visualizer is destroyed when the parent drone object is removed, preventing "phantom waypoints" from cluttering the scene.

.. tip::
   For best visibility in VR, use a high-emission material on the ``targetSphere`` so it glows. If the path line appears jagged, ensure the ``LineRenderer`` is set to use **World Space** coordinates in the Unity Inspector.


.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Installation
*******************************

This guide covers the steps required to set up the Unity simulation environment. 

.. note::
   The Python-based **DroneManager** backend is a separate installation. Please refer to the :doc:`installation` guide for instructions on setting up the Python environment and its dependencies.

Prerequisites
=============

Before setting up the Unity project, ensure you have the following installed:

* **Unity Hub & Unity 6000.2.8f1:** The project uses the Universal Render Pipeline (URP).
* **Unity Android Build Support:** Essential for Meta Quest VR development. In Unity Hub, ensure the following modules are added to your Editor version:
    * **Android Build Support**
    * **Android SDK & NDK Tools**
    * **OpenJDK**
* **Visual Studio 2022:** Must be installed with the **"Game development with Unity"** workload (includes C# and required Unity integration tools).
* **Git:** Required to clone the repository and manage any submodules.

Setting Up the Unity Project
============================

1. Clone the Repository
-----------------------

Clone the project from GitHub using your terminal or Git client:

.. code-block:: bash

   git clone https://github.com/YourUsername/Holodeck-Unity.git
   cd Holodeck-Unity

2. Open via Unity Hub
---------------------

* Launch **Unity Hub**.
* Click **Add** > **Add project from disk**.
* Navigate to and select the ``Holodeck-Unity`` folder.
* Ensure the Editor version is set to *6000.2.8f1**.

3. Install Dependencies
-----------------------

Once the project is open, verify the following in the **Package Manager** (Window > Package Manager):

* **Newtonsoft Json.NET:** If not present, click the "+" icon, select "Add package by name," and enter ``com.unity.nuget.newtonsoft-json``.
* **XR Interaction Toolkit:** (Optional) Required for VR functionality. Ensure this is installed and the "Oculus" or "OpenXR" provider is enabled in **Project Settings > XR Plug-in Management**.


Usage
*******************************

Setting Up a Custom Holodeck Scene
==================================

While the project provides a pre-configured ``ExampleScene.unity``, you may want to create a custom Digital Twin of your specific flight environment. Follow these steps to integrate Holodeck logic into a new scene.

1. Scene Preparation
--------------------
* Create a new Unity Scene.
* Ensure your environment scale is **1:1** (1 Unity unit = 1 Meter) to match the real-world GPS/Local coordinates.
* Add a ground plane and any static obstacles (trees, buildings).

2. Core Logic Integration
-------------------------
* **Manager Object:** Create an empty GameObject named ``[Holodeck_Manager]``.
* **Attach Scripts:** Drag and drop the ``UDPReceiver.cs`` and ``DroneManager.cs`` scripts onto this object.
* **Configure Ports:** In the ``UDPReceiver`` inspector, set the ``serverIp`` and ``serverPort`` to match your Python backend.

3. Camera and VR Setup
----------------------
* Delete the default "Main Camera."
* Drag the ``OVRCameraRig`` (or your preferred XR Rig) into the scene.
* Attach the ``CameraSpringArm.cs`` to the Rig's center eye or root.
* **Important:** Link this Rig to the ``Follow Camera`` slot in the ``DroneManager`` component.

4. Drone Prefab Assignment
--------------------------
* Locate the ``Drone_Prefab`` in your Assets folder. 
* Ensure the prefab has the ``DroneController.cs``, ``FenceDisplay.cs``, and ``TargetDisplay.cs`` components attached.
* Drag this prefab into the ``Drone Prefab`` slot of your ``DroneManager``.

Running the Simulation
======================

Once the scene is set up, follow this execution order:

1.  **Start the Backend:** Run your DroneManager Python script. It will begin listening for a connection request.
2.  **Enter Play Mode:** Press **Play** in the Unity Editor.
3.  **Handshake:** The ``UDPReceiver`` will automatically send a start request. Once received, the Python backend will begin streaming telemetry.
4.  **Verification:** Check the Unity Console. You should see: 
    ``"CHECK 3: First UDP data packet received! Processing drones..."``

.. tip::
   If you want to quickly test your setup without creating a scene from scratch, open ``Assets/Scenes/SampleScene``. It contains a fully functional configuration with a demo environment and all script references already linked.

