
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

## Start A star solver with memray and save result to logger.log and action_log.csv

```bash
# terminal 1
cd ~/mdl_ros2_ws/src/mdl_robot/mdl_control
memray run --live-remote scripts/planner.py
```

```bash
# terminal 2
memray live <port number>
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
