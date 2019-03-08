# Gate Simulator
*Goal* : Synthetically create training data for fylable region detection.
Should practically run on Mac, Linux and Windows but tested throrougly only on Ubuntu16.04 and 18.04
Probably doesn't require Unity to be installed.

# Instructions for Executing 
* Download & extract the zip. Then `cd` into GateSim folder.
* Execute `chmod +x SimulatedData_x86.64` to give executable permission (if needed).
* Ensure that there is no `results.txt` and that `SimulatedData_Data/screenshots` either is empty or doesn't exist
* Run the binary `./SimulatedData.x86_64`

# Usage
* `WASD` to move in plane. `RightShift` and `CapsLock` for Down and Up, mouse to change direction. 
* Press `I` or `P` to change background (currently either of the 2 DRL logos)
* Press `U` to change the lights of the spotlights projected on our gate.
* Press `K` ensuring flyable region is visible to you. A screenshot will be saved in required dimensions same as given training data in 'GateSim/SimulatedData_Data/screenshots'. Corresponding co-ordinates will be stored in 'results.txt'
* For burst mode (like video) please press `V`. Everything else is same as above bullet.

# Postprocess
* Added a simple python script to load images and manipulate the generated results and create a json out of it
* See more in `ppsim.py` 
* Bounding box order = LowerLeft, LowerRight, UpperRight, UpperLeft
* There are different 

# Caveats
* The pixels are approximate and not 100% accurate. There is about 5-10% error.
