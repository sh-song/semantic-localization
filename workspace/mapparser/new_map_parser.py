import json
import argparse
from tqdm import tqdm

import pymap3d

import numpy as np
from ngiiParser import NGIIParser

class MapElementExtractor:


    def __init__(self, map_path, base_lla, precision):

        self.ngii = self._set_NGII_parser(map_path)
        self.base_lla = list(map(float, base_lla))
        self.precision = precision
        

    def _to_enu(self, lat, lon, alt):
        return pymap3d.geodetic2enu(lat, lon, alt, self.base_lla[0], self.base_lla[1], self.base_lla[2])


    def _set_NGII_parser(self, map_path):

        ## Load map files
        a1_path = '%s/A1_NODE.shp'%(map_path)
        a2_path = '%s/A2_LINK.shp'%(map_path)
        a3_path = '%s/A3_DRIVEWAYSECTION.shp'%(map_path)
        a4_path = '%s/A4_SUBSIDIARYSECTION.shp'%(map_path)

        b1_path = '%s/B1_SAFETYSIGN.shp'%(map_path)
        b2_path = '%s/B2_SURFACELINEMARK.shp'%(map_path)
        b3_path = '%s/B3_SURFACEMARK.shp'%(map_path)

        c1_path = '%s/C1_TRAFFICLIGHT.shp'%(map_path)
        c3_path = '%s/C3_VEHICLEPROTECTIONSAFETY.shp'%(map_path)
        c4_path = '%s/C4_SPEEDBUMP.shp'%(map_path)
        c6_path = '%s/C6_POSTPOINT_Merge.shp'%(map_path)

        ngii = NGIIParser(
            a1_path,
            a2_path,
            a3_path,
            a4_path,
            b1_path, 
            b2_path, 
            b3_path, 
            c1_path,
            c3_path,
            c4_path,
            c6_path)

        return ngii

    def _extract_points(self, id_of_interest):
        points_dict = {}
        for b2_surfacelinemark in tqdm(self.ngii.b2_surfacelinemark, desc="b2_surfacelinemark: ", total=len(self.ngii.b2_surfacelinemark)):

            if b2_surfacelinemark.ID in id_of_interest:

                id_ = b2_surfacelinemark.ID

                length = len(b2_surfacelinemark.geometry.coords)
                points = np.zeros([3, length])
                for i, (lon, lat, alt) in enumerate(b2_surfacelinemark.geometry.coords):
                    points[:, i]  = np.array(self._to_enu(lat, lon, alt))

                points_dict[id_] = points

                
        return points_dict

    def _interpolate_points(self, points):
        pass

    def run(self):


        id_of_interest = ['B2225A110799','B2225A110800']
        points_dict = self._extract_points(id_of_interest)
        
        print(points_dict)



if __name__ == "__main__":

    ## Parse Argument
    parser = argparse.ArgumentParser(description='Get lat, lon, alt as arguments in tuples')
    parser.add_argument('--ngii_path', required=True, help='Path to the NGII map')
    parser.add_argument('--precision', required=True, help='Precision value for the map')
    parser.add_argument('--base_lla', required=True, nargs=3, help='Latitude, longitude, and altitude of the base')

    args = parser.parse_args()
    map_path=args.ngii_path
    precision = float(args.precision)

    extractor = MapElementExtractor(map_path=map_path,
                                    base_lla=args.base_lla,
                                    precision=precision)
    
    extractor.run()

    #self.generate_lanelet(ngii, precision)
    map_data = {}
    lanelets = {}

    to_node = {}
    from_node = {}

    side_lanelets = {} # outer lines
    stoplines = {}
    safetysigns = {}
    surfacemarks = {}
    trafficlights = {}
    vehicleprotectionsafetys = {}
    speedbumps = {}
    postpoints = {}
