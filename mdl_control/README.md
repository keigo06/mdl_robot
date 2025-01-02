
# mdl_control

## setup

- need to install setup.py for pytest

```bash
cd ~/mdl_ros2_ws
python3 -m venv .venv
. .venv/bin/activate
cd src/mdl_robot/mdl_control
pip install -e .
```

## tutorial

```bash
# terminal 1
~/mdl_ros2_ws$ ros2 launch　mdl_control display_server.launch.py
```

## pytest

```bash
cd ~/mdl_ros2_ws
. .venv/bin/activate
cd cd src/mdl_robot/mdl_control
pytest -v -s
```

```bash
cd ~/mdl_ros2_ws
. .venv/bin/activate
cd cd src/mdl_robot/mdl_control
pytest tests/test_cube.py
```
