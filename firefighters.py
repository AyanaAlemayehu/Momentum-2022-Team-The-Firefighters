from shapely.geometry import Polygon
from student_base import student_base
import time
import numpy
import json

class my_flight_controller(student_base):
	"""
	Student flight controller class.

	Students develop their code in this class.

	Parameters
	----------
	student_base : student_base
		Class defining functionality enabling base functionality
	
	Methods
	-------
	student_run(self, telemetry: Dict, commands: Dict (optional))
		Method that takes in telemetry and issues drone commands.
	"""

	def student_run(self, telemetry, commands):
		
		# The telemetry dictionary contains fields that describe the drone's position and flight state.
		# It updates continuously, so it can be polled for new information.
		# Use a time.sleep() between polls to keep the CPU load down and give the background communications
		# a chance to run.
		bash = open("launch_px4_boston.bash", "r")
		i = 0
		home_coords = []
		for line in bash:
			if i in range(6,9):
				home_coords.append(line[line.index("=")+1:].strip("\n"))
			i+=1
		print(home_coords)

		# finding coordinates of fires
		fires_raw = open("maps/boston_fire.json", "r")
		fires_str = json.load(fires_raw)["data_fs"]
		fires_polygon_verticies = []
		fire_coordsx, fire_coordsy = [], []
		for xs in fires_str["xs"]:
			fire_coordsx.append(sum(xs)/len(xs))
			fires_polygon_verticies.append(xs)
		i = 0
		for ys in fires_str["ys"]:
			fires_polygon_verticies[i] = zip(fires_polygon_verticies[i], ys)
			fire_coordsy.append(sum(ys)/len(ys))
			i += 1
		fire_coords = list(zip(fire_coordsx, fire_coordsy))
		print(fire_coords)
		print(fires_polygon_verticies)
		
		fires_shapes = []
		for fire_verticies in fires_polygon_verticies:
			fires_shapes.append(Polygon(fire_verticies).area)

		print(fires_shapes)

		i=0
		fire_size_to_coordinates = {}
		for shapes in fire_shapes:
			fire_size_to_coordinates[fire_coords[i]] = shape
			i+=1

	
# This bit of code just makes it so that this class actually runs when executed from the command line,
# rather than just being silently defined.

if __name__ == "__main__":
	fcs = my_flight_controller()
	fcs.run()
