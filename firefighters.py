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
	
	adj_matrix = []
	fire_size_to_coordinates = {}
	
	#incorporate different speeds for different directions, other considerations like fire size and water proximity
	#fire_centers is a list of len 2 tuples
	def fillAdjMatrix(self, fire_centers):
		for fire in range(len(fire_centers)):
			for other_fire in range(len(fire_centers)):
				distance = ((fire_centers[fire][0] - fire_centers[other_fire][0])**2 + (fire_centers[fire][1] - fire_centers[other_fire][1])**2)**.5
				#distance can be tailored depending on fire attributes in the future
				self.adj_matrix[other_fire][fire] = distance
					

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
		fire_centers = list(zip(fire_coordsx, fire_coordsy))
		print(fire_centers)
		print(fires_polygon_verticies)
		
		
		#filling adjacency matrix with None values
		self.adj_matrix = [[None for i in range(len(fire_centers))] for j in range(len(fire_centers))]
		
		
		fires_shapes = []
		for fire_verticies in fires_polygon_verticies:
			fires_shapes.append(Polygon(fire_verticies).area)

		print(fires_shapes)

		#generating dictionary that holds information for each fire, where the fire averaged centers are keys
		i=0
		for shapes in fires_shapes:
			self.fire_size_to_coordinates[fire_centers[i]] = shapes
			i+=1
			
		self.fillAdjMatrix(fire_centers)
		print(self.adj_matrix)
		
		#find a way to mark a fire as complete
		#code a function to reduce a fire to a given tolerance
		#code a function to grab water when out of water
		#get full tank on every pickup except for the end of the game


	
# This bit of code just makes it so that this class actually runs when executed from the command line,
# rather than just being silently defined.

if __name__ == "__main__":
	fcs = my_flight_controller()
	fcs.run()
