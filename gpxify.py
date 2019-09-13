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
import douglaspeucker as dp

#from dateutil import parser
import time

class Gpxify:

    @staticmethod
    def print_status(msg, *args, **kwargs):
        print("{} - {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S"), msg), *args, **kwargs)

    def __init__(self, fn):
        self.fn = fn
        self.points = self.load_points()

    def parse_points(self, gpx):
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


    def mark_peaks(self):

        distances, elevations, elevations_smooth = ([] for i in range(3))
        for point in self.points:
            distances.append(point.distance/1000)
            elevations.append(point.elevation)
            elevations_smooth.append(point.elevation_smooth)

        elevations_smooth = np.array(elevations_smooth)
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

        for index_h in hills:
            index = bisect.bisect_left(valleys, index_h)-1
            if index != -1:
                index_v = valleys[index]
                #print("{}->{}".format(index_v, index_h))
                elevation = elevations[index_h] - elevations[index_v]
                # suppress small hills
                if elevation > 20:
                    self.points[index_v].type = "valley"
                    self.points[index_v].desc =  "{elevation}m/{distance}km".format(
                        elevation = round(elevation),
                        distance = round(distances[index_h] - distances[index_v], 1)
                    )
                    self.points[index_h].type = "hill"
                    self.points[index_h].desc = "{}m".format(round(elevations[index_h]))

                del valleys[index]


    def print_peaks(self):
        for peak in self.get_peaks():
            print("{distance:>5}km: {desc:<12} {type}".format(
                distance = round(peak.distance/1000, 1),
                desc = peak.desc,
                type = peak.type
            ))

    def get_peaks(self, type=None):
        return [ w for w in self.points if getattr(w, "type", type) ]


    def plot_peaks(self, fig):
        for peak in self.get_peaks():
            distance = peak.distance/1000
            fig.plot(distance, peak.elevation, "r.")
            fig.annotate(
                peak.desc,
                fontsize="small",
                color="k",
                xy=(distance, peak.elevation),
                textcoords="offset points",
                xytext=(0,3 if peak.type == "hill" else -10),
                ha="center" if peak.type == "hill" else "left"
            )

    def get_summary(self):
        """ get summary of track as string """

        return "distance: {}km, time: {}, avg speed: {}km/h".format(
            round(self.points[-1].distance / 1000, 1),
            time.strftime("%H:%M:%S", time.gmtime(self.points[-1].time)),
            round(3.6 * self.points[-1].distance / self.points[-1].time, 1)
        )

    def load_points(self):
        gpx = gpxpy.parse(open(self.fn))

        return self.parse_points(gpx)

    def add_smooth_elevation(self):
        """
        add smoothed elevation to points
        """
        elevations_smooth = savgol_filter([point.elevation for point in self.points], 101, 2)
        for point, elevation_smooth in zip(self.points, elevations_smooth):
            point.elevation_smooth = elevation_smooth

    def plot_tracks(self, fig, reduced):
        fig.plot([p.lat for p in self.points], [p.lon for p in self.points], "-", color="silver", linewidth=8.0, alpha=.7, label="original")
        fig.plot([p.lat for p in reduced], [p.lon for p in reduced], "-k", label="reduced")

        for peak in self.get_peaks():
            fig.plot(peak.lat, peak.lon, "r.")
            fig.annotate(
                peak.desc,
                fontsize="small",
                color="r",
                xy=(peak.lat, peak.lon),
                textcoords="offset points",
                xytext=(0,3),
                ha="left"
            )


        fig.set_xlabel("{}/{} point reduction".format(len(reduced), len(self.points)))
        fig.legend(loc=2)

    def plot_elevation(self, fig):
        distances, elevations, elevations_smooth = ([] for i in range(3))
        has_smooth = True if self.points[0].elevation_smooth else False

        for p in self.points:
            distances.append(p.distance/1000)
            elevations.append(p.elevation)
            if has_smooth:
                elevations_smooth.append(p.elevation_smooth)

        fig.plot(distances, elevations, "-", color="silver", linewidth=4.0, alpha=.7, label="original")
        if has_smooth:
            fig.plot(distances, elevations_smooth, "-r", linewidth=.5, label="smooth")

        self.plot_peaks(fig)

        fig.set_ylabel("Elevation in m")
        fig.set_xlabel("Distance in km")
        fig.legend(loc=2)

    def plot_speed(self, fig):
        slopes, speeds = ([] for i in range(2))

        for p in self.points:
            slopes.append(p.slope)
            speeds.append(p.speed*3.6)

        fig.axis([-20, 20, 0, 60])
        fig.set_ylabel("Speed in km/h")
        fig.set_xlabel("Slope in %")
        #fig.scatter(slopes, speeds)
        fig.plot(slopes, speeds, "bo", markersize=1)

    def plot(self):
        Gpxify.print_status("Hello")

        print(self.get_summary())

        # smoothen elevation with savitzky-golay
        self.add_smooth_elevation()

        # get hills, valleys
        # https://stackoverflow.com/questions/37598986/reducing-noise-on-data
        self.mark_peaks()
        self.print_peaks()

        has_speed = True if self.points[0].speed else False
        fig, ax = plt.subplots(nrows=1, ncols=3 if has_speed else 2, squeeze=False)

        self.plot_elevation(ax[0,0])

        # reduce data with douglas peucker
        reduced_points = dp.reduce(self.points)
        self.plot_tracks(ax[0,1], reduced_points)

        if has_speed:
            self.plot_speed(ax[0,2])

        Gpxify.print_status("Done.")
        #ax[0].plot([distances[peak] for peak in hills], [elevations[hill] for hill in hills], ".g", label="top")
        #ax[0].plot([distances[valley] for valley in valleys], [elevations[valley] for valley in valleys], "xg", label="bottom")
        # ax[0].plot([distances[valley] for valley in valleys[0]], elevations_smooth[valleys], "xg", label="bottom")
        #ax[0].plot(distances_reduced, elevations_reduced, "-", color="gray", linewidth=1.0, label="reduced")

        plt.show()