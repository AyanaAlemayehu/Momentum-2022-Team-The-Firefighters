from shapely.geometry import Point, Polygon
from student_base import student_base
from shapely.ops import nearest_points
from shapely.geometry import LinearRing


import time
import numpy
import json
import geopandas as gpd


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
	
	#index using [start][end]
	adj_matrix = None

	runFlag = False
	initial_size_to_coords = {}
	initial_total_area = 0
	total_fire_area = 0
	#regularly updated dicitonary
	fire_size_to_coordinates = {}


	water_sources_poly = []
	telem = None
	


	def move(self, x, y):
		print("MOVING TO: " + str((x,y)))
		point = (x,y)
		err = numpy.linalg.norm([point[0] - self.telem['latitude'], point[1] - self.telem['longitude']])
		tol = 0.00001 # Approximately 50 feet tolerance
		self.goto(x, y, 100)
		while err > tol:
			err = numpy.linalg.norm([point[0] - self.telem['latitude'], point[1] - self.telem['longitude']])

	#incorporate different speeds for different directions, other considerations like fire size and water proximity
	#fire_centers is a list of len 2 tuples
	def fillAdjMatrix(self, fire_centers, tolerance):
		#clearing adjacency matrix before editing
		self.adj_matrix = [[float("inf") for i in range(len(fire_centers) + 1)] for j in range(len(fire_centers) + 1)]
		#tolerance is how large the fire has to be for us to consider it
		#for example, 10 means its the top 10% of fires in size
		#attempted adding our own drone to adjacency matrix as well
		fire_centers.append((self.telem["longitude"], self.telem["latitude"]))
		for fire in range(len(fire_centers)):
			for other_fire in range(len(fire_centers) - 1):
				if fire == other_fire:
					continue
				if fire == len(fire_centers) - 1:
					#if fire is indexed to the drone
					if (self.telem["fire_polygons"][other_fire].area/self.total_fire_area >= tolerance*.01):
						#if the other fire is large enough, calculate distance between the drone and the other fire
						distance = ((fire_centers[fire][0] - fire_centers[other_fire][0])**2 + (fire_centers[fire][1] - fire_centers[other_fire][1])**2)**.5
						#distance can be tailored depending on fire attributes in the future
						self.adj_matrix[fire][other_fire] = distance
				#else its fire to fire
				else:
					if (self.telem["fire_polygons"][fire].area/self.total_fire_area >= tolerance*.01 and self.telem["fire_polygons"][other_fire].area/self.total_fire_area >= tolerance*.01):
						#than we consider it
						distance = ((fire_centers[fire][0] - fire_centers[other_fire][0])**2 + (fire_centers[fire][1] - fire_centers[other_fire][1])**2)**.5
						#distance can be tailored depending on fire attributes in the future
						self.adj_matrix[fire][other_fire] = distance
				#else its fire to drone so we dont care
		print("NEW ADJACENCY MATRIX")
		print(self.adj_matrix)
	def nearestWater(self, point):
		dist = [poly.distance(point) for poly in self.water_sources_poly]
		min_index = dist.index(min(dist))
		p1, p2 = nearest_points(self.water_sources_poly[min_index].boundary, point)
		#return nearest point rounded to 5 decimal poitns
		return (round(p1.x, 5), round(p1.y, 5))

	def fillWater(self):
		point = self.nearestWater(Point(float(self.telem["longitude"]), float(self.telem["latitude"])))
		self.move(point[1], point[0])
		#now that drone has arrived, wait until water pct is 100%
		print("FILLING WATER")
		while self.telem["water_pct_remaining"] < 100:
			print(self.telem["water_pct_remaining"])
			time.sleep(1)
			pass

	

	# def initFire(self):
	# 	# finding original coordinates of fires, also populating fire centers
	# 	fires_raw = open("maps/boston_fire.json", "r")
	# 	fires_json = json.load(fires_raw)
	# 	fires_str = fires_json["data_fs"]
	# 	fires_polygon_verticies = []
	# 	fire_coordsx, fire_coordsy = [], []
	# 	for xs in fires_str["xs"]:
	# 		fire_coordsx.append(sum(xs)/len(xs))
	# 		fires_polygon_verticies.append(xs)
	# 	i = 0
	# 	for ys in fires_str["ys"]:
	# 		fires_polygon_verticies[i] = zip(fires_polygon_verticies[i], ys)
	# 		fire_coordsy.append(sum(ys)/len(ys))
	# 		i += 1
	# 	fire_centers = list(zip(fire_coordsx, fire_coordsy))
	# 	#filling adjacency matrix with None values (added one to include space for drone)
	# 	self.adj_matrix = [[None for i in range(len(fire_centers) + 1)] for j in range(len(fire_centers) + 1)]
	# 	fires_shapes = []
	# 	for fire_verticies in fires_polygon_verticies:
	# 		fires_shapes.append(Polygon(fire_verticies).area)

	# 	self.fires_initial = fires_shapes
	# 	#print(fires_shapes)

	# 	#generating dictionary that holds information for each fire, where the fire averaged centers are keys
	# 	i=0
	# 	for shapes in fires_shapes:
	# 		self.fire_size_to_coordinates[fire_centers[i]] = shapes
	# 		i+=1

	def updateFires(self):
		#currently updates the fire_size_to_coords dicitonary to update each fires midpoint and area
		temp = []
		for fire_poly in range(len(self.telem["fire_polygons"])):
			#grabbing x and y values
			xsum = ysum = 0
			for coord in self.telem["fire_polygons"][fire_poly].exterior.coords[:-1]:
				xsum += coord[0]
				ysum += coord[1]
			xsum /= (len(self.telem["fire_polygons"][fire_poly].exterior.coords)-1)
			ysum /= (len(self.telem["fire_polygons"][fire_poly].exterior.coords)-1)
			temp.append((xsum, ysum))
			if not self.runFlag:
				self.initial_total_area += self.telem["fire_polygons"][fire_poly].area
				self.initial_size_to_coords[fire_poly] = [self.telem["fire_polygons"][fire_poly].area, (xsum, ysum)]
			self.fire_size_to_coordinates[fire_poly] = [self.telem["fire_polygons"][fire_poly].area, (xsum, ysum)]
		self.runFlag = True

		#setting total area to equal initial area here
		self.total_fire_area = self.initial_total_area

		if self.adj_matrix == None:
			#added one to include the drone in the matrix
			self.adj_matrix = [[float("inf") for i in range(len(temp) + 1)] for j in range(len(temp) + 1)]

			

	def student_run(self, telemetry, commands):
		self.telem = telemetry

		print(telemetry)
		print(" ")
		print(" ")
		# The telemetry dictionary contains fields that describe the drone's position and flight state.
		# It updates continuously, so it can be polled for new information.
		# Use a time.sleep() between polls to keep the CPU load down and give the background communications
		# a chance to run.

		# bash = open("launch_px4_boston.bash", "r")
		# i = 0
		#HOME COORDS IS FLIPPED
		
		self.arm()
		

		fires_raw = open("maps/boston_fire.json", "r")
		fires_json = json.load(fires_raw)
		
		#every time we refill the tank OR we finish a fire, re-assess the situation and possibly switch objectives

		#home_coords = [telemetry["latitude"], telemetry["longitude"]]
		
		# for line in bash:
		# 	if i in range(6,9):
		# 		home_coords.append(line[line.index("=")+1:].strip("\n"))
		# 	i+=1

		self.updateFires()
			
		#print(self.adj_matrix)
		


		#find a way to mark a fire as complete
		#code a function to reduce a fire to a given tolerance
		#code a function to grab water when out of water
		#get full tank on every pickup except for the end of the game


		#attempting to get water data

		bbox = (fires_json["bounds"]["maxx"][0], fires_json["bounds"]["maxy"][0], fires_json["bounds"]["minx"][0], fires_json["bounds"]["miny"][0])
		bounding_polygon = Polygon([(bbox[0], bbox[1]), 
                            (bbox[0], bbox[3]), 
                            (bbox[2], bbox[3]), 
                            (bbox[2], bbox[1]), 
                            (bbox[0], bbox[1])])

		rivers = gpd.read_file('data/ne_10m_rivers_north_america.geojson', 
							bbox=bbox, 
							crs="EPSG:4326")
		lakes = gpd.read_file('data/ne_10m_lakes.geojson', 
							bbox=bbox, 
							crs="EPSG:4326")
		usa_states = gpd.read_file('data/cb_2018_us_state_20m.zip', 
								bbox=bbox, 
								crs="EPSG:4326")
		oceans = gpd.read_file('data/ne_10m_ocean_scale_rank.geojson', 
							bbox=bbox, 
							crs="EPSG:4326")


		# Clean up data
		rivers = rivers[rivers.featurecla != 'Lake Centerline']
		rivers = rivers[rivers.featurecla != 'Lake Centerline (Intermittent)']
		oceans = gpd.clip(oceans, bounding_polygon).to_crs("EPSG:4326")

		# Combine waterbody data
		waterbodies = rivers.append(lakes)
		waterbodies = waterbodies.append(oceans)

		#print(waterbodies)
		waterbodies.to_file("waterbodies.geojson", driver='GeoJSON')
		
		#converting waterbodies into polygons
		for raw_poly in json.loads(waterbodies.to_json())["features"][0]["geometry"]["coordinates"]:
			self.water_sources_poly.append(Polygon(raw_poly[0]))
		
		#first nearest water call that uses home base as point

		#first water source:
		print("going to first water source")
		self.takeoff()
		print("Waiting 6 seconds")
		time.sleep(6)
		
		# water
		print("Go to first water source")

		#heart of algorithm
		#as time decreases, having a preference over a fire closer to water may be worth it

		#first refill adjacency matrix after new position from water refill
		print("\n")
		self.fillAdjMatrix([fire[1] for fire in self.fire_size_to_coordinates.values()], 20)
		print(self.adj_matrix)
		print(self.fire_size_to_coordinates)


		#maybe find better condition but white true works for now
		while True:
			print("running")
			if telemetry["water_pct_remaining"] == 0:
				print("STARTING TO GRAB WATER")
				self.fillWater()
			#now put out the closest member of adjacency matrix
			next_fire = self.adj_matrix[len(self.adj_matrix) - 1].index(min(self.adj_matrix[len(self.adj_matrix) - 1]))
			print("THIS IS THE NEXT FIRE: " + str(next_fire))
			point = self.fire_size_to_coordinates[next_fire][1]
			self.move(point[1], point[0])


			#CONTINOUSLY RECALCULATING TOTAL FIRE TO ACCOMODATE FOR CHANGING ENVIRONMENT
			self.total_fire_area = self.initial_total_area*.01*telemetry["fires_pct_remaining"]

			self.updateFires()
			self.fillAdjMatrix([fire[1] for fire in self.fire_size_to_coordinates.values()], 20)
			#1. get warer
			#2. find a fire
			#3. put out fire


		#TODO
		#FIND OUT WHY DRONE PAUSES (could be a physics thing, could be a thread sleeping thing, etc. office could help)
		#FIND OUT WHY DRONE PRIORITIZES WRONG FIRES
		#WHY DOES DRONE FULLY TURN OFF FIRE
		#why does the second triangle not trigger but the third one early?
		#WHY DOES IT WAIT A RANDOM AMONUT OF TIME BEFORE EXECUTING MOVE COMMANDS (increases as game prgoresses)
		#possible solution grabs the largest fire and normalizes all the distance using that


		#NOT IMPORTANT PATHS IN ADJACENCY MATRIX ARE LEFT NONETYPE
		

		

				

# This bit of code just makes it so that this class actually runs when executed from the command line,
# rather than just being silently defined.

if __name__ == "__main__":
	fcs = my_flight_controller()
	fcs.run()
