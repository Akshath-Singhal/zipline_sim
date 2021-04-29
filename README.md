# zipline_sim

The simulator has been developed by Zipline.
The autoppilot governing navigation, collision avoidance and payload drop has been implemented in my_pilot.py

Use the following commands to run the simulator and autopilot:

'''shell
python3 -m venv test_environment
source test_environment/bin/activate
pip3 install -r requirements.txt
python3 zip_sim.py python my_pilot.py
'''
