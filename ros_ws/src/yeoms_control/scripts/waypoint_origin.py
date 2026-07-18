#!/usr/bin/env python3
import csv
import os
import sys


def main():
    if len(sys.argv) != 2:
        print("usage: waypoint_origin.py <waypoint_csv>", file=sys.stderr)
        return 2

    path = os.path.expanduser(sys.argv[1])
    with open(path, newline="") as fp:
        reader = csv.DictReader(fp)
        first = next(reader, None)

    if not first:
        print("origin_lat:=0.0 origin_lon:=0.0")
        return 0

    lat = first.get("origin_lat") or first.get("lat") or "0.0"
    lon = first.get("origin_lon") or first.get("lon") or "0.0"
    alt = first.get("origin_alt") or first.get("alt") or "0.0"
    print(f"origin_lat:={lat} origin_lon:={lon} origin_alt:={alt}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
