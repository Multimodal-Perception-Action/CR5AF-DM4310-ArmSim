# Third-party notices

This distribution bundles third-party software and model assets. Each component remains under its own license. This project does not replace or override those terms.

## Python runtime and wheels

- Python 3.13.9 Windows installer: Python Software Foundation License. Source and license: <https://www.python.org/downloads/> and <https://docs.python.org/3/license.html>.
- The `wheels/` directory contains the packages pinned in `requirements-runtime.txt`. Wheel metadata and publisher-supplied license files remain inside each `.whl` archive under its `.dist-info` directory where provided.
- Bundled packages include ftservo-python-sdk, pyserial, NumPy, MuJoCo, trimesh, pycollada, SciPy, Pillow, glfw, PyOpenGL, websockets, python-dateutil, six, esptool and its pinned dependencies. Review each wheel's `METADATA` and license files before redistributing it separately.

## Robot and model assets

- Dobot `TCP-IP-ROS-6AXis-models` assets retain the upstream license at `official/TCP-IP-ROS-6AXis-models/LICENSE`.
- PathOn Robotics assets retain `LICENSE-APACHE-2.0` and `NOTICE` in their copied asset directory.
- LeRobot Anything U-Arm sources retain `models/source/LeRobot-Anything-U-Arm/LICENSE.txt` and the accompanying upstream README files.
- Curated `uarm_cr10a_teleop_sim` sources retain their original notices and documentation where present. A model without an explicit license should not be redistributed separately without permission.

## Original project code

No standalone license for the original project code has been declared in this release. Publication and download do not grant additional reuse rights beyond applicable law and the separate third-party licenses above. A future release may add a project license after the owner selects one.
