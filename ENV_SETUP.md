# AUBO_VLA_Project Environment

Current stage:
AUBO Sim + JSON-RPC + Robot Backend + Fake VLA pipeline

Install robot backend dependencies inside .venv:

python3 -m pip install -r requirements_robot.txt

Main test:

python3 tests/test_vla_pipeline.py

Project structure:
robot/       AUBO communication and driver
control/     action representation and decoding
vla/         VLA policy interface
tests/       verification scripts
