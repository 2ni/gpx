import argparse
from gpxify import Gpxify


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analizing gpx', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-s', '--speed', type=float, default=27.0, help='Speed in km/h for tcx file')
    parser.add_argument('-m', '--min_size_peak', type=float, default=20.0, help='Ignore any peak below this threshold')
    parser.add_argument("file", type=str, nargs="?", help="file to analyze")
    # parser.add_argument("-i", "--ids", action="store_true", help="1) get ids")

    args = parser.parse_args()
    args.file = args.file or "4012568328.gpx"

    gpxify = Gpxify(fn=args.file, speed=args.speed, min_size_peak=args.min_size_peak)
    gpxify.plot()
