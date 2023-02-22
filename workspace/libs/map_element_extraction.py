import numpy as np

import pymap3d


from libs.ngiiParser import NGIIParser
from libs.map_elements_db import MapElementsDB

class MapElementExtractor:

    def __init__(self, cfg):
        self.cfg = cfg

        ngii = self._get_parsed_NGII_map(cfg.map_path)
        # self.base_lla = list(map(float, cfg.base_lla))
        self.base_lla = cfg.base_lla
        self.map_db = MapElementsDB(ngii, self.base_lla)

        self.precision = cfg.precision
        
    def _to_enu(self, lat, lon, alt):
        return pymap3d.geodetic2enu(lat, lon, alt, self.base_lla[0], self.base_lla[1], self.base_lla[2])


    def _get_parsed_NGII_map(self, map_path):

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


    def _extract_points(self, target_element : str, linkID_of_interest : list):
        points_dict = {}

        for current_linkID in linkID_of_interest:

            for id_, coords in self.map_db.link_dict[current_linkID][target_element].items():

                length = len(coords)
                points = np.zeros([3, length])
 
                for i, (lon, lat, alt) in enumerate(coords):
                    points[:, i]  = np.array(self._to_enu(lat, lon, alt))

                points_dict[id_] = points

        return points_dict


    def _interpolate_points(self, points):
        pass

    def _get_id_of_interest(self, current_linkID : str):
        id_of_interest = [current_linkID]

        ## CURRENT NEXT
        for next_linkID in self.map_db.link_dict[current_linkID]['NEXT']:
            id_of_interest.append(next_linkID)

        ## CURRENT PREV
        for prev_linkID in self.map_db.link_dict[current_linkID]['PREV']:
            id_of_interest.append(prev_linkID)
        
        ## CURRENT LEFT
        for left_linkID in self.map_db.link_dict[current_linkID]['LEFT']:
            id_of_interest.append(left_linkID)

        ## LEFT NEXT
            for next_linkID in self.map_db.link_dict[left_linkID]['NEXT']:
                id_of_interest.append(next_linkID)

        ## LEFT PREV
            for prev_linkID in self.map_db.link_dict[left_linkID]['PREV']:
                id_of_interest.append(prev_linkID)
    
        ## CURRENT RIGHT
        for right_linkID in self.map_db.link_dict[current_linkID]['RIGHT']:
            id_of_interest.append(right_linkID)

        ## RIGHT NEXT
            for next_linkID in self.map_db.link_dict[right_linkID]['NEXT']:
                id_of_interest.append(next_linkID)

        ## RIGHT PREV
            for prev_linkID in self.map_db.link_dict[right_linkID]['PREV']:
                id_of_interest.append(prev_linkID)
 
        return id_of_interest

    def _generate_straight_lane(self, x, isVertical=True):
        length = 20
        n = 50
        if isVertical:
            xs = np.ones(n) * x
            ys = np.linspace(0, 10, n)
        else: #is Horizontal
            xs = np.linspace(0, 10, n)
            ys = np.ones(n) * x
 
        zs = np.ones(n) * 0
        ones = np.ones(n)

        elem = np.hstack([xs, ys, zs, ones]).reshape(4, n)
        return elem

    def run(self, element_names, vehicle_pose_WC):
        
        if self.map_db.isQueryInitialized:
            current_linkID = self.map_db.get_current_linkID(vehicle_pose_WC)
        else: # not initialized, not use previous linkID
            current_linkID = self.map_db.initialize_query(vehicle_pose_WC)

        elems_WC_dict = {}
        elems_length_dict = {}

        linkID_of_interest = self._get_id_of_interest(current_linkID)

        for target_element in element_names:
            
            if target_element == "HELP":
                print("Available elements: LINEMARK, TRAFFICLIGHT")

            if target_element in ["LINEMARK", "TRAFFICLIGHT", "SAFETYSIGN", "SURFACEMARK"]:
                points_dict = self._extract_points(target_element, linkID_of_interest)
                tmp_points = None
                for points in points_dict.values():
                    if tmp_points is not None:
                        tmp_points = np.hstack([tmp_points, points])
                    else: ## First time
                        tmp_points = points
                
                if tmp_points is not None:
                    length = tmp_points.shape[1]
                    elems_WC_dict[target_element] = np.vstack([tmp_points, np.ones(length)])
                    elems_length_dict[target_element] = length
 


            elif target_element == "VIRTUAL LINEMARK":

                elements_list = []

                left_lane_WC = self._generate_straight_lane(4)
                right_lane_WC = self._generate_straight_lane(6)

                elements_list.append(left_lane_WC)
                elements_list.append(right_lane_WC)

                front_far_lane_WC = self._generate_straight_lane(10, isVertical=False)
                front_near_lane_WC = self._generate_straight_lane(8, isVertical=False)
                elements_list.append(front_far_lane_WC)
                elements_list.append(front_near_lane_WC)

                right_new_lane_WC = self._generate_straight_lane(8)
                elements_list.append(right_new_lane_WC)
                
                elems_WC_dict[target_element] = np.hstack(elements_list)
        #end for
        return elems_WC_dict, elems_length_dict


