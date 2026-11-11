# Hardware and data path

## Bench topology

The tested gripper data path was:

PC application -> Ethernet/TCP port 60000 -> Dobot CC262 tool-RS485 channel -> T-CAN485 RS485 input -> Classic CAN -> DM-J4310-2EC

The motor uses its own 24 V supply. The PC application originates the request bytes. The CC262 provides the tool-serial transport; the gateway converts the request to a CAN frame and returns received data.

A separate U-Arm path is:

U-Arm FE-URT-2 -> USB -> Windows PC

The PC can combine leader input with robot and gripper interfaces in software. Unplugging the USB leader does not make the robot generate commands; another host-side command source or controller-side application would be needed.

## Interface boundaries

- CR5AF joint motion and status use the robot controller's supported interfaces.
- The tool-RS485 byte tunnel is a transport path, not a CAN port.
- T-CAN485 performs the RS485/CAN physical and protocol conversion.
- DM-J4310-2EC uses CAN. RS485 A/B must never be wired directly to CAN_H/CAN_L.
- The bench result does not establish a user-accessible native CAN API on CC262.

## What the byte-path test proves

A fixed byte sequence sent by the PC was observed at the T-CAN485 RS485 receiver. A separate read-only DM register/status request returned data through the gateway. These observations verify those specific paths and request types; they do not validate a general-purpose control bridge or a production safety function.