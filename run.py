import argparse
import sys
import re
import os
from datetime import datetime, timedelta

import matplotlib.pyplot as plt

import gpxpy
import gpxpy.gpx
import gpxpy.geo

#from dateutil import parser
import time

class Gpxify:

    @staticmethod
    def print_status(msg, *args, **kwargs):
        print("{} - {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S"), msg), *args, **kwargs)

    def __init__(self):
        self.fn = "4012568328.gpx"

    def main(self):
        parser = argparse.ArgumentParser(description='Analizing gpx', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.add_argument("-f", "--file", type=str, const=self.fn, nargs="?", help="file to analyze")
        parser.add_argument("-i", "--ids", action="store_true", help="1) get ids")

        Gpxify.print_status("Hello")

        gpx = gpxpy.parse(open(self.fn), "r")

        # douglas peucker
        gpx.simplify(max_distance=10)

        number_of_points = 0
        total_ascent = 0
        total_distance = 0
        timeline = 0
        previous_point = None
        distance = None

        # data to plot
        speeds = []
        angles = []

        timelines = []
        elevations = []
        distances = []

        # do not count waiting time in calculation
        # stopping creates just a new segment
        for track in gpx.tracks:
            for segment in track.segments:
                previous_point = None
                for point in segment.points:
                    number_of_points += 1

                    if previous_point:
                        # distance = gpxpy.geo.length([previous_point, point])
                        distance = point.distance_2d(previous_point)
                        ascent = previous_point.elevation - point.elevation
                        angle = previous_point.elevation_angle(point, radians=False)
                        speed = 3.6*(point.speed + previous_point.speed)/2
                        time_delta = (point.time - previous_point.time).total_seconds()

                        speeds.append(speed)
                        angles.append(angle)

                        if ascent > 0:
                            total_ascent += ascent
                        total_distance += distance
                        timeline += time_delta

                        timelines.append(timeline)
                        elevations.append(point.elevation)
                        distances.append(round(total_distance/1000,1))

                        print("{}/{}: {}m {}m ({}km/h @ {}° in {}s)".format(
                            round(point.latitude, 3),
                            round(point.longitude, 3),
                            round(distance, 1),
                            round(ascent, 1),
                            round(speed, 1),
                            round(angle, 1),
                            time_delta
                        ))

                    previous_point = point

        print("{} points processed".format(number_of_points))
        print("distance: {}km, ascent: {}m".format(
            round(total_distance/1000, 1),
            round(total_ascent))
        )
        print(str(timedelta(seconds=timeline)))

        plt.figure()
        plt.subplot(121)
        plt.plot(angles, speeds, "ro", markersize=1)
        plt.axis([-20, 20, 0, 60])
        plt.ylabel("Speed in km/h")
        plt.xlabel("Angle in °")
        plt.subplot(122)
        """
        plt.plot(timelines, elevations, "-b")
        plt.ylabel("Elevation in m")
        plt.xlabel("Time in sec")
        """
        plt.plot(distances, elevations, "-b", linewidth=1.0)
        plt.ylabel("Elevation in m")
        plt.xlabel("Distance in km")
        plt.show()

if __name__ == '__main__':
    gpxify = Gpxify()
    gpxify.main()
