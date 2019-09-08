import argparse
from gpxify import Gpxify


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analizing gpx', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-f", "--file", type=str, nargs="?", help="file to analyze")
    parser.add_argument("-i", "--ids", action="store_true", help="1) get ids")

    args = parser.parse_args()
    args.file = args.file or "4012568328.gpx"

    gpxify = Gpxify()
    gpxify.plot(args.file)
