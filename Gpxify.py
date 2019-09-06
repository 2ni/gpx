import argparse
import sys
import re
import os
import math
import bisect
from datetime import datetime, timedelta

import matplotlib.pyplot as plt
from scipy.signal import lfilter, savgol_filter, argrelextrema, find_peaks
import numpy as np

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
        pass

    def extract(self, gpx):
        number_of_points = 0
        total_ascent = 0
        total_distance = 0
        timeline = 0
        previous_point = None
        distance = None

        # data to plot
        slope_speeds = []
        distance_elevations = []

        # do not count waiting time in calculation
        # stopping creates just a new segment
        for track in gpx.tracks:
            for segment in track.segments:
                previous_point = None
                for point in segment.points:
                    number_of_points += 1

                    if previous_point:
                        # print([attr for attr in dir(point) if not attr.startswith('__')])
                        # distance = gpxpy.geo.length([previous_point, point])
                        distance = previous_point.distance_2d(point)
                        ascent = previous_point.elevation - point.elevation
                        angle = previous_point.elevation_angle(point, radians=False)
                        time_delta = (point.time - previous_point.time).total_seconds()
                        if point.speed and previous_point.speed:
                            speed = 3.6*(point.speed + previous_point.speed)/2
                            slope = math.tan(math.radians(angle))*100
                            slope_speeds.append((slope, speed))

                        distance_elevations.append((round(total_distance/1000,1), point.elevation))

                        if ascent > 0:
                            total_ascent += ascent
                        total_distance += distance
                        timeline += time_delta


                        """
                        print("{}/{}: {}m {}m ({}km/h @ {}°/{}% in {}s)".format(
                            round(point.latitude, 3),
                            round(point.longitude, 3),
                            round(distance, 1),
                            round(ascent, 1),
                            round(speed, 1),
                            round(angle, 2),
                            round(slope, 1),
                            time_delta
                        ))
                        """

                    previous_point = point

        print("{} points processed".format(number_of_points))
        print("distance: {}km, ascent: {}m".format(
            round(total_distance/1000, 1),
            round(total_ascent))
        )
        print(str(timedelta(seconds=timeline)))

        return slope_speeds, distance_elevations

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

    def main(self):
        parser = argparse.ArgumentParser(description='Analizing gpx', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.add_argument("-f", "--file", type=str, nargs="?", help="file to analyze")
        parser.add_argument("-i", "--ids", action="store_true", help="1) get ids")

        args = parser.parse_args()
        args.file = args.file or "4012568328.gpx"

        Gpxify.print_status("Hello")

        gpx = gpxpy.parse(open(args.file))
        slope_speeds, distance_elevations = self.extract(gpx)
        distances, elevations = list(zip(*distance_elevations))

        # smoothen elevation with savitzky-golay
        # algorithms return indexes where extremas are found
        # https://stackoverflow.com/questions/37598986/reducing-noise-on-data
        elevations_smooth = savgol_filter(elevations, 101, 2)

        # get hills, valleys
        peaks = self.get_peaks(distances, elevations, elevations_smooth)
        self.print_peaks(distances, peaks)

        # reduce data with douglas peucker
        gpx.simplify(max_distance=10)
        d_slope_speeds, d_distance_elevations = self.extract(gpx)
        distances_reduced, elevations_reduced = list(zip(*d_distance_elevations))

        # plot 1st graph (slope/speed)
        plt.figure()
        if len(slope_speeds):
            plt.subplot(121)
            plt.axis([-20, 20, 0, 60])
            plt.ylabel("Speed in km/h")
            plt.xlabel("Slope in %")
            slopes, speeds = list(zip(*slope_speeds))
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

if __name__ == '__main__':
    gpxify = Gpxify()
    gpxify.main()
