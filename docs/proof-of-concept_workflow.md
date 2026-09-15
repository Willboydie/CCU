1. Sessions
The device begins Out Of Session when the program first runs. For a session to begin, a UI must send a BEGIN_SESSION command packet. In Session, all telemetry is enabled and the full functionality
of the device is unlocked. When a session is ended by UI request (END_THIS_SESSION), internal session data is wiped and defaults reinstated. Turning the device power off will cause a Session to end; Session data will not be retained in non-volatile memory.

2. Frontend Connection
When disconnected, the device heartbeats a DeviceID packet through the wifi chip. The packet includes an id string, device displayName and the sessionStatus.
The UI application, once launched, will listen to the wifi dongle for incoming device IDs. If one is recognised, the UI can request to either join the current session (if sessionStatus=Active) or start a new one.
Upon receipt of a session join or begin request, the device will latch into a Connected state and cease to send DeviceID packets.
The UI considers itself connected when it has received a Regular Telemetry packet in the last 5s. While connected, it will send Connected packets at 10Hz. After 5s of not receiving a Connected packet from the frontend, the device will latch into an disconnected state and restart the transmission of DeviceID packets. A disconnection event does not trigger an End of Session, this can only be done via UI request.

3. Idle and Running
To leave Idle, the system must receive from the UI a SET_ORIGIN command (setting the current pose of the device as the plan origin) 
--and-- a REGISTER_NODES command packet, which contains the East-North coordinates and corner orientations of the building plan vertices.
Once the session has registered the receipt of an origin and a set of plan nodes, it is Running and any of the *Macro Routines* can be requested.

4. Macro Routines
The system performs the following Macro Routines:

**Tag Finding.** The laser is activated on a low duty cycle and the program responds to *Nudges* from the UI which increment the angle of the laser beam.
The UI reports "Tag 


5. Regular Telemetry
When In Session, regardless of connection status, the device will transmit RegularTelemetry packets at 10Hz. These packets contain:
- **(Pose) lastKnownPose**. The last registered pose of the device.
- **(float) effectiveYaw**. The yaw of the device with respect to the vertical (up) axis. Is computed via lastKnownPose and baseRotationAngle.
- **(bool) moved**. A latching state which defaults to false and becomes true in response to IMU readings showing movement deemed not to be base rotation.
- **(bool) originIsSet**. false until the UI sends a SET_ORIGIN packet. Reverts back to false upon a CLEAR_ORIGIN command or a Session ending.
- **(SurveyStatus) surveyStatus**. Either IDLE, SCANNING or READY. SCANNING and READY states are signals from the device to the UI during the Survey macro routine.
- **(ProjectorStatus) projectorStatus**. Either IDLE, COMPUTING_PATH or PROJECTING. Signals to the UI during a Projection macro routine.
- **(bool) laserOn**. Signals laser emission status.
- **(int) laserPower**. The set duty cycle of the laser.

6. Tag Finding Telemetry

7. Survey Telemetry

8. Projection Telemetry