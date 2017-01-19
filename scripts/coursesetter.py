import json, os, gmplot

class Courses:
	def __init__(self, coursefp='../courses.json', waypointfp='../waypoints.json'):
		self.coursefp = coursefp
		self.waypointfp = waypointfp

		if (os.path.isfile(coursefp)):
			with open(self.coursefp, 'r') as coursefile:
				self.courses = json.load(coursefile)

		else:
			self.courses = {}
			self.save()

		if (os.path.isfile(waypointfp)):
			with open(self.waypointfp, 'r') as waypointfile:
				self.waypoints = json.load(waypointfile)

		else:
			self.waypoints = {}

	def listwpts(self):
		print "Waypoints: " + str([str(wpt) for wpt in self.waypoints.keys()])

	def listcourses(self):
		print "Courses: " + str([str(course) for course in self.courses.keys()])

	def add(self, name, waypoints):
		self.courses[name] = [(
			wpt,
			self.waypoints[wpt]["latitude"],
			self.waypoints[wpt]["longitude"]
		) for wpt in waypoints]
		self.plot(name)
		self.save()

	def delete(self, name):
		if name in self.courses:
			del self.courses[name]

		filepath = "../documentation/coursemaps/" + name + ".html"

		if (os.path.isfile(filepath)):
			os.remove(filepath)

		self.save()

	def save(self):
		with open(self.coursefp, 'w') as coursefile:
			json.dump(self.courses, coursefile, sort_keys=True, indent=2, separators=(',', ": "))

	def plot(self, name):
		gmap = gmplot.GoogleMapPlotter(
			42.29335375,
			-71.26358725,
			19)

		latitudes = [float(wpt[1]) for wpt in self.courses[name]]
		longitudes = [float(wpt[2]) for wpt in self.courses[name]]

		gmap.scatter(latitudes, longitudes, '#000000', marker=True);
		gmap.draw("../documentation/coursemaps/"+ name +".html");


if __name__ == "__main__":
	c = Courses()

	gps_single = ['gps0' + str(digit) for digit in range(1,10)]
	gps_double = ['gps' + str(digit) for digit in range(10,31)]
	#c.delete('Google');
	c.add('Google', gps_single + gps_double)

	c.listcourses()