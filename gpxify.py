import sys
import re
import os
import math
import bisect
from datetime import datetime, timedelta
from ppretty import ppretty

import matplotlib.pyplot as plt
from scipy.signal import lfilter, savgol_filter, argrelextrema, find_peaks
import numpy as np

import gpxpy
import gpxpy.gpx
import gpxpy.geo

from bunch import Bunch

#from dateutil import parser
import time

class Gpxify:

    @staticmethod
    def print_status(msg, *args, **kwargs):
        print("{} - {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S"), msg), *args, **kwargs)

    def __init__(self):
        pass

    def get_points(self, gpx):
        """
        extract points from gpx
        append some major information to each point
        """
        points = []
        total_distance = 0
        total_time = 0
        total_ascent = 0
        previous_point = None

        for track in gpx.tracks:
            for segment in track.segments:
                previous_point = None
                for point in segment.points:
                    if previous_point:
                        total_distance += previous_point.distance_2d(point)
                        total_time += (point.time - previous_point.time).total_seconds()
                        angle = previous_point.elevation_angle(point, radians=True)
                        ascent = previous_point.elevation - point.elevation
                        if ascent > 0:
                            total_ascent += ascent

                        p = Bunch(
                            lat = point.latitude,
                            lon = point.longitude,
                            elevation = point.elevation,
                            slope = round(math.tan(angle)*100, 1), # in %
                            distance = round(total_distance),      # in meter
                            time = total_time,                     # in seconds
                            speed = point.speed,                   # in m/s
                        )
                        points.append(p)

                    previous_point = point

        return points


    def get_peaks(self, distances, elevations, elevations_smooth):
        #peaks, _ = find_peaks(elevations_smooth, distance=100)
        #plt.plot([distances[peak] for peak in peaks], elevations_smooth[peaks], ".r", label="find_peaks")

        number_of_points = len(elevations_smooth)
        number_of_points_to_consider = round(number_of_points/50)

        hills = list(argrelextrema(elevations_smooth, np.greater, order=number_of_points_to_consider)[0])
        valleys = list(argrelextrema(elevations_smooth, np.less, order=number_of_points_to_consider)[0])

        # always start with a valley
        if hills[0] < valleys[0]:
            valleys.insert(0, 0)

        # always end with a hill
        if valleys[-1] > hills[-1]:
            del valleys[-1]

        waypoints = {}
        for index_h in hills:
            index = bisect.bisect_left(valleys, index_h)-1
            if index != -1:
                index_v = valleys[index]
                #print("{}->{}".format(index_v, index_h))
                elevation = elevations[index_h] - elevations[index_v]
                # suppress small hills
                if elevation > 20:
                    waypoints[index_v] = "{elevation}m/{distance}km".format(
                        elevation = round(elevation),
                        distance = round(distances[index_h] - distances[index_v], 1)
                    )
                    waypoints[index_h] = "{}m".format(round(elevations[index_h]))

                del valleys[index]

        return waypoints

    def print_peaks(self, distances, peaks):
        i=0
        for index, peak in peaks.items():
            print("{index:>5} ({distance:>5} km): {peak:<12} {type}".format(
                index = index,
                peak = peak,
                distance = distances[index],
                type = "valley" if (i % 2) == 0 else "hill"
            ))
            i += 1


    def plot_peaks(self, distances, elevations, peaks):
        i=0
        for index, peak in peaks.items():
            type = "valley" if (i % 2) == 0 else "hill"
            plt.plot(distances[index], elevations[index], "r.")
            plt.annotate(
                peak,
                fontsize="small",
                color="k",
                xy=(distances[index], elevations[index]),
                textcoords="offset points",
                xytext=(0,3 if type == "hill" else -10),
                ha="center" if type == "hill" else "left"
            )
            i += 1

    def plot(self, fn):
        Gpxify.print_status("Hello")

        gpx = gpxpy.parse(open(fn))
        points = self.get_points(gpx)

        distances, elevations, slopes, speeds = ([] for i in range(4))
        has_speed = True if points[0].speed else False
        for point in points:
            distances.append(point.distance/1000)
            elevations.append(point.elevation)
            if has_speed:
                slopes.append(point.slope)
                speeds.append(point.speed*3.6)


        # smoothen elevation with savitzky-golay
        # algorithms return indexes where extremas are found
        # https://stackoverflow.com/questions/37598986/reducing-noise-on-data
        elevations_smooth = savgol_filter(elevations, 101, 2)

        # get hills, valleys
        peaks = self.get_peaks(distances, elevations, elevations_smooth)
        self.print_peaks(distances, peaks)

        # reduce data with douglas peucker
        #gpx.simplify(max_distance=10)

        # plot 1st graph (slope/speed)
        plt.figure()
        if has_speed:
            plt.subplot(121)
            plt.axis([-20, 20, 0, 60])
            plt.ylabel("Speed in km/h")
            plt.xlabel("Slope in %")
            plt.plot(slopes, speeds, "bo", markersize=1)

            # plot 2 graph (elevation)
            plt.subplot(122)

        plt.plot(distances, elevations, "-", color="silver", linewidth=4.0, alpha=.7, label="original")
        plt.plot(distances, elevations_smooth, "-r", linewidth=.5, label="smooth")
        #plt.plot([distances[peak] for peak in hills], [elevations[hill] for hill in hills], ".g", label="top")
        #plt.plot([distances[valley] for valley in valleys], [elevations[valley] for valley in valleys], "xg", label="bottom")
        # plt.plot([distances[valley] for valley in valleys[0]], elevations_smooth[valleys], "xg", label="bottom")
        #plt.plot(distances_reduced, elevations_reduced, "-", color="gray", linewidth=1.0, label="reduced")

        self.plot_peaks(distances, elevations, peaks)

        plt.ylabel("Elevation in m")
        plt.xlabel("Distance in km")
        plt.legend(loc=2)

        plt.show()
