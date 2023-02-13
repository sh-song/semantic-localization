import sys
import math
import utm
import pymap3d
import numpy as np
from tqdm import tqdm

from ngiiParser import NGIIParser
from utils import *


class NGII2LANELET:
    def __init__(self, 
        folder_path,
        precision,
        base_lla,
        is_utm):

        a1_path = '%s/A1_NODE.shp'%(folder_path)
        a2_path = '%s/A2_LINK.shp'%(folder_path)
        a3_path = '%s/A3_DRIVEWAYSECTION.shp'%(folder_path)
        a4_path = '%s/A4_SUBSIDIARYSECTION.shp'%(folder_path)

        b1_path = '%s/B1_SAFETYSIGN.shp'%(folder_path)
        b2_path = '%s/B2_SURFACELINEMARK.shp'%(folder_path)
        b3_path = '%s/B3_SURFACEMARK.shp'%(folder_path)

        c1_path = '%s/C1_TRAFFICLIGHT.shp'%(folder_path)
        c3_path = '%s/C3_VEHICLEPROTECTIONSAFETY.shp'%(folder_path)
        c4_path = '%s/C4_SPEEDBUMP.shp'%(folder_path)
        c6_path = '%s/C6_POSTPOINT_Merge.shp'%(folder_path)

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

        self.base_lla = base_lla
        self.is_utm = is_utm
        self.generate_lanelet(ngii, precision)

    def to_cartesian(self, tx, ty, alt=None):
        if self.is_utm:
            lat, lon = utm.to_latlon(tx, ty, alt, 'N')
        else:
            lat, lon = ty, tx

        if self.base_lla is None:
            self.base_lla = (lat, lon, alt)

        if alt is None:
            x, y, _ = pymap3d.geodetic2enu(lat, lon, self.base_lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y
        else:
            x, y, z = pymap3d.geodetic2enu(lat, lon, alt, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y, z

    def generate_lanelet(self, ngii, precision):
        self.map_data = {}
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


        for a2_link in tqdm(ngii.a2_link, desc="a2_link: ", total=len(ngii.a2_link)):
            id_ = a2_link.ID

            lanelets[id_] = {}

            waypoints = []
            for tx, ty, alt in a2_link.geometry.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                waypoints.append((x, y, z))

            waypoints, s, yaw, k = interpolate(waypoints, precision)

            lanelets[id_]['waypoints'] = waypoints
            lanelets[id_]['idx_num'] = len(waypoints)
            lanelets[id_]['yaw'] = yaw
            lanelets[id_]['s'] = s
            lanelets[id_]['k'] = k
            lanelets[id_]['length'] = s[-1] # a2_link.Length
            lanelets[id_]['laneNo'] = a2_link.LaneNo
            lanelets[id_]['leftBound'] = []
            lanelets[id_]['leftType'] = []
            lanelets[id_]['rightBound'] = []
            lanelets[id_]['rightType'] = []

            if to_node.get(a2_link.ToNodeID) is None:
                to_node[a2_link.ToNodeID] = []

            to_node[a2_link.ToNodeID].append(id_)

            if from_node.get(a2_link.FromNodeID) is None:
                from_node[a2_link.FromNodeID] = []

            from_node[a2_link.FromNodeID].append(id_)

            if a2_link.LinkType == '1':
                lanelets[id_]['intersection'] = True
            else:
                lanelets[id_]['intersection'] = False
 
        for a2_link in ngii.a2_link:
            id_ = a2_link.ID

            if not lanelets[id_]['intersection']:
                lanelets[id_]['adjacentLeft'] = a2_link.L_LinKID
                lanelets[id_]['adjacentRight'] = a2_link.R_LinkID
            else:
                lanelets[id_]['adjacentLeft'] = None
                lanelets[id_]['adjacentRight'] = None

            lanelets[id_]['predecessor'] = to_node[a2_link.FromNodeID] if to_node.get(a2_link.FromNodeID) is not None else []
            lanelets[id_]['successor'] = from_node[a2_link.ToNodeID] if from_node.get(a2_link.ToNodeID) is not None else []

        for b1_safetysign in tqdm(ngii.b1_safetysign, desc="b1_safetysign: ", total=len(ngii.b1_safetysign)):
            obj_id = b1_safetysign.ID

            points = []

            for tx, ty, alt in b1_safetysign.geometry.exterior.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                points.append((x, y, z))

            safetysigns[obj_id] = points

        for b2_surfacelinemark in tqdm(ngii.b2_surfacelinemark, desc="b2_surfacelinemark: ", total=len(ngii.b2_surfacelinemark)):
            
            if b2_surfacelinemark.Kind  != '530':
                id_ = b2_surfacelinemark.R_linkID

                if b2_surfacelinemark.geometry is not None:
                    leftBound = []
                    for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                        x, y, z = self.to_cartesian(tx, ty, alt)
                        leftBound.append((x, y, z))

                    leftBound, s, yaw, k = interpolate(leftBound, precision)

                    if len(leftBound) > 1:
                        if lanelets.get(id_) is not None:
                            lanelets[id_]['leftBound'].append(leftBound)
                            lanelets[id_]['leftType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                        else:
                            side_lanelets[id_] = [leftBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted']

                id_ = b2_surfacelinemark.L_linkID

                if b2_surfacelinemark.geometry is not None:
                    rightBound = []
                    for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                        x, y, z = self.to_cartesian(tx, ty, alt)
                        rightBound.append((x, y, z))

                    rightBound, s, yaw, k = interpolate(rightBound, precision)

                    if len(rightBound) > 1:
                        if lanelets.get(id_) is not None:
                            lanelets[id_]['rightBound'].append(rightBound)
                            lanelets[id_]['rightType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                        else:
                            side_lanelets[id_] = [rightBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted']

            else: # stop line
                points = []
                for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                    x, y, z = self.to_cartesian(tx, ty, alt)
                    points.append((x, y, z))

                stoplines[id_] = points

        for b3_surfacemark in tqdm(ngii.b3_surfacemark, desc="surfacemark: ", total=len(ngii.b3_surfacemark)):
            obj_id = b3_surfacemark.ID

            points = []

            for tx, ty, alt in b3_surfacemark.geometry.exterior.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                points.append((x, y, z))

            surfacemarks[obj_id] = points


        for c1_trafficlight in tqdm(ngii.c1_trafficlight, desc="trafficlight: ", total=len(ngii.c1_trafficlight)):
            obj_id = c1_trafficlight.ID

            
            tx, ty, alt = list(c1_trafficlight.geometry.coords)[0]
            x, y, z = self.to_cartesian(tx, ty, alt)

            trafficlights[obj_id] = (x, y, z)

        for c3_vehicleprotectionsafety in tqdm(ngii.c3_vehicleprotectionsafety, desc="vehicleprotectionsafety: ", total=len(ngii.c3_vehicleprotectionsafety)):
            obj_id = c3_vehicleprotectionsafety.ID

            points = []

            for tx, ty, alt in c3_vehicleprotectionsafety.geometry.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                points.append((x, y, z))

            vehicleprotectionsafetys[obj_id] = points
        
        for c4_speedbump in tqdm(ngii.c4_speedbump, desc="speedbump: ", total=len(ngii.c4_speedbump)):
            obj_id = c4_speedbump.ID

            points = []

            for tx, ty, alt in c4_speedbump.geometry.exterior.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                points.append((x, y, z))

            speedbumps[obj_id] = points
        
        for c6_postpoint in tqdm(ngii.c6_postpoint, desc="trafficlight: ", total=len(ngii.c6_postpoint)):
            obj_id = c6_postpoint.ID

            tx, ty, alt = list(c6_postpoint.geometry.coords)[0]
            x, y, z = self.to_cartesian(tx, ty, alt)

            postpoints[obj_id] = (x, y, z)

        self.map_data['base_lla'] = self.base_lla
        self.map_data['precision'] = precision
        self.map_data['lanelets'] = lanelets
        self.map_data['side_lanelets'] = side_lanelets
        self.map_data['stoplines'] = stoplines
        self.map_data['safetysigns'] = safetysigns
        self.map_data['surfacemarks'] = surfacemarks
        self.map_data['trafficlights'] = trafficlights
        self.map_data['vehicleprotectionsafetys'] = vehicleprotectionsafetys
        self.map_data['speedbumps'] = speedbumps
        self.map_data['postpoints'] = postpoints