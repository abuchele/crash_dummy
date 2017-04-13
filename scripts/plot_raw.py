import os, re, string, sys, json, gmplot

SRCFILE = "./testdata/gps_test.txt"
PLOTNAME = "gps_test"

latitudes = []
longitudes = []

with open(SRCFILE, "r") as file:
	# wptname = el[:-4]

	for line in file:
		if re.search('latitude', line):
			print line
			latitudes.append(float(line[10:-1]))

		if re.search('longitude', line):
			longitudes.append(float(line[11:-1]))

	# wpts[wptname] = {
	# 	"longitude": avglong / longcnt,
	# 	"latitude": avglat / latcnt
	# }
# print wptname
# print "lat: " + str(avglat / latcnt)
# print "long: " + str(avglong / longcnt)

gmap = gmplot.GoogleMapPlotter(
42.29335375,
-71.26358725,
19)

# latitudes = [float(wpt[1]) for wpt in self.courses[name]]
# longitudes = [float(wpt[2]) for wpt in self.courses[name]]

print latitudes

gmap.scatter(latitudes, longitudes, '#000000', marker=True);
gmap.draw("../documentation/testdata/"+ PLOTNAME +".html");