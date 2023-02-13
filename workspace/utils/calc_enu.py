
import pymap3d
import argparse
def calc_enu(lat, lon, alt, base_lla):
    return pymap3d.geodetic2enu(lat, lon, alt, base_lla[0], base_lla[1], base_lla[2])


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Get lat, lon, alt as arguments in tuples')
    parser.add_argument('--lat', required=True)
    parser.add_argument('--lon', required=True)
    parser.add_argument('--alt', required=True)

    base_lla = [37.3890294,126.6487574,6.0]

    args = parser.parse_args()

    lat = float(args.lat)
    lon = float(args.lon)
    alt = float(args.alt)

    x,y,z = calc_enu(lat, lon, alt, base_lla)

    print(f"\n x: {x} y: {y} z: {z}")
    print(f"\nFor copy:\n{x}, {y}, {z}")