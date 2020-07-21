"""
get data from vomhof: http GET https://www.hofsuche.vomhof.ch/api/de/farms|json_pp>vomhof.json
process to gpx which then can be used in qmapshack as overlay
"""

import gpxpy
import gpxpy.gpx
import json

gpx = gpxpy.gpx.GPX()
gpx.name = "vomhof"

with open("vomhof.json") as js:
    data = json.load(js)

for hof in data["aggregations"]["mapLocations"]["buckets"]:
    gpx_wps = gpxpy.gpx.GPXWaypoint()
    gpx_wps.latitude = hof["_source"]["location"]["lat"]
    gpx_wps.longitude = hof["_source"]["location"]["lon"]
    gpx_wps.name = hof["_source"]["farmname"]
    gpx_wps.symbol = "Pin, Blue"
    gpx_wps.description = "{street}, {zip} {city} | https://hofsuche.vomhof.ch/de/farm/{id}".format(
        id=hof["_id"],
        street=hof["_source"]["street"],
        zip=hof["_source"]["zip"],
        city=hof["_source"]["city"]
    )
    gpx.waypoints.append(gpx_wps)
    print(hof["_id"], hof["_source"]["farmname"])

with open("vomhof.gpx", "w") as fn:
    fn.write(gpx.to_xml())
