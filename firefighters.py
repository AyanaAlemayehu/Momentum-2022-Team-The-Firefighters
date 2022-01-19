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
	
	adj_matrix = []
	fire_size_to_coordinates = {}
	water_sources_poly = []
	
	#incorporate different speeds for different directions, other considerations like fire size and water proximity
	#fire_centers is a list of len 2 tuples
	def fillAdjMatrix(self, fire_centers):
		for fire in range(len(fire_centers)):
			for other_fire in range(len(fire_centers)):
				distance = ((fire_centers[fire][0] - fire_centers[other_fire][0])**2 + (fire_centers[fire][1] - fire_centers[other_fire][1])**2)**.5
				#distance can be tailored depending on fire attributes in the future
				self.adj_matrix[other_fire][fire] = distance

	def nearestWater(self, point):
		dist = [poly.distance(point) for poly in self.water_sources_poly]
		min_index = dist.index(min(dist))
		p1, p2 = nearest_points(self.water_sources_poly[min_index].boundary, point)
		print("NEAREST WATER POINT (old): " + str(p1.wkt) + "  " + str(p2.wkt))		
		print("DISTANCE: " + str(min(dist)))			
		pol_ext = LinearRing(self.water_sources_poly[min_index].exterior.coords)
		d = pol_ext.project(point)
		p = pol_ext.interpolate(d)
		print("NEW NEAREST WATER POINNT: " + str(p.wkt))





		triangle = Polygon((0, 0), (1, 0), (0.5, 1), (0, 0)])
		square = Polygon([(0, 2), (1, 2), (1, 3), (0, 3), (0, 2)])
		print([o.wkt for o in nearest_points(triangle, square)])


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
		#print(home_coords)

		# finding coordinates of fires
		fires_raw = open("maps/boston_fire.json", "r")
		fires_json = json.load(fires_raw)
		fires_str = fires_json["data_fs"]
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
		#print(fire_centers)
		#print(fires_polygon_verticies)
		
		
		#filling adjacency matrix with None values
		self.adj_matrix = [[None for i in range(len(fire_centers))] for j in range(len(fire_centers))]
		
		
		fires_shapes = []
		for fire_verticies in fires_polygon_verticies:
			fires_shapes.append(Polygon(fire_verticies).area)

		#print(fires_shapes)

		#generating dictionary that holds information for each fire, where the fire averaged centers are keys
		i=0
		for shapes in fires_shapes:
			self.fire_size_to_coordinates[fire_centers[i]] = shapes
			i+=1
			
		self.fillAdjMatrix(fire_centers)
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
		print(json.loads(waterbodies.to_json())["features"][0]["geometry"]["coordinates"])
		for raw_poly in json.loads(waterbodies.to_json())["features"][0]["geometry"]["coordinates"]:
			print("RAW POLYGON: " + str(raw_poly[0]))
			self.water_sources_poly.append(Polygon(raw_poly[0]))
			print(Polygon(raw_poly[0]).exterior.coords.xy)
		
		#first nearest water call that uses home base as point
		print(home_coords)
		self.nearestWater(Point(float(home_coords[0]), float(home_coords[1])))
		
		
		
		

		

				

# This bit of code just makes it so that this class actually runs when executed from the command line,
# rather than just being silently defined.

if __name__ == "__main__":
	fcs = my_flight_controller()
	fcs.run()
