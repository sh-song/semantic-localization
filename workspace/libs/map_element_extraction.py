import numpy as np

from tqdm import tqdm

import pymap3d


from libs.ngiiParser import NGIIParser
from scipy.spatial import cKDTree

class MapElementsDB:
    def __init__(self, ngii : NGIIParser, base_lla):
        self.base_lla = base_lla
        self.link_KDT_metadata = {}
        self.link_refpoints = []

        nodeIDs = self._get_nodeIDs(ngii.a1_node)
        self.link_start_from = self._set_node_dict(nodeIDs)
        self.link_end_at = self._set_node_dict(nodeIDs)
        self.link_dict = self._set_link_dict(ngii.a2_link)
        self.link_KDTree = self._get_KDTree(self.link_refpoints)
    
        for nodeID in nodeIDs:

            for linkID in self.link_start_from[nodeID]:
                self.link_dict[linkID]['PREV'] = \
                                self.link_end_at[nodeID]

            for linkID in self.link_end_at[nodeID]:
                self.link_dict[linkID]['NEXT'] = \
                                self.link_start_from[nodeID]

        self._parse_elements_to_link("SAFETYSIGN", ngii.b1_safetysign)
        self._parse_elements_to_link("LINEMARK", ngii.b2_surfacelinemark)
        self._parse_elements_to_link("SURFACEMARK", ngii.b3_surfacemark)
        self._parse_elements_to_link("TRAFFICLIGHT", ngii.c1_trafficlight)


        self.isQueryInitialized = False
        self.prev_linkID = None

    def _get_nodeIDs(self, a1_node):
        nodeIDs = []
        for node in a1_node:
            nodeIDs.append(node.ID)
        return nodeIDs
 
    def _set_node_dict(self, nodeIDs : list):
        node_dict = {}
        for nodeID in nodeIDs:
            node_dict[nodeID] = []
        return node_dict


    def _set_link_dict(self, a2_link):
        link_dict = {}
        for link in a2_link:

            ## Init link dict
            link_dict[link.ID] = self._get_standard_dict()

            ## Add Adjacent LinkIDs
            if type(link.R_LinkID) is str:
                link_dict[link.ID]['RIGHT'].append(link.R_LinkID)

            if type(link.L_LinKID) is str:
                link_dict[link.ID]['LEFT'].append(link.L_LinKID)

            ## Add Next & Prev NodeIDs
            self.link_start_from[link.FromNodeID].append(link.ID)
            self.link_end_at[link.ToNodeID].append(link.ID)

            ## Add Reference Points (lon, lat, alt)
            kdt_idx0 = len(self.link_KDT_metadata.keys())

            ### Add First Coordinate (lon, lat, alt)
            self.link_refpoints.append(link.geometry.coords[0])
            self.link_KDT_metadata[str(kdt_idx0)] = link.ID

            ### Add Middle Coordinate (lon, lat, alt)
            mid_idx = len(link.geometry.coords) // 2
            self.link_refpoints.append(link.geometry.coords[mid_idx])
            self.link_KDT_metadata[str(kdt_idx0+1)] = link.ID
 
            ### Add Last Coordinate (lon, lat, alt)
            self.link_refpoints.append(link.geometry.coords[-1])
            self.link_KDT_metadata[str(kdt_idx0+2)] = link.ID

        return link_dict

    def _get_standard_dict(self):
        return {'NEXT':[], 'PREV':[], 'LEFT': [], 'RIGHT': [], 
                'MIDPOINT': ('lon', 'lat', 'alt'), 
                'LINEMARK': {}, 'TRAFFIC SIGN': {}}

    def _get_KDTree(self, points:list):
        tmp = np.array(points)        
        tmp = pymap3d.geodetic2enu(tmp[:, 1], tmp[:, 0], tmp[:, 2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
        tmp = np.array(tmp)[0:2, :] ## eliminate altitude
        return cKDTree(tmp.T)

    def _parse_elements_to_link(self, target_name : str, ngii_elements : list):
        for element in ngii_elements:

            if target_name == "LINEMARK":
                if type(element.L_linkID) is str: #not NaN
                    self.link_dict[element.L_linkID][target_name][element.ID] = element.geometry.coords

                if type(element.R_linkID) is str:
                    self.link_dict[element.R_linkID][target_name][element.ID] = element.geometry.coords

            ## TODO: Implement other elements
            if target_name == "SAFETYSIGN":
                pass

            if target_name == "TRAFFICLIGHT":
                pass

            if target_name == "SURFACEMARK":
                pass

    def initialize_query(self, vehicle_pose_WC):
        x = vehicle_pose_WC[0:2]
        dists, idxs = self.link_KDTree.query(x, k=10, p=2, workers=1)

        print(f"\n===============\nNearest Link Query from {x}\n")
        for i, idx in enumerate(idxs):
            print(f"Rank#{i}   idx: {idx}  dist: {dists[i]} link: {self.link_KDT_metadata[str(idx)]}")
        print("\n===================")

        current_linkID = self.link_KDT_metadata[str(idxs[0])]
        self.isQueryInitialized = True
        self.prev_linkID = current_linkID
        return current_linkID

    def get_current_linkID(self, vehicle_pose_WC):

        ##TODO: find current linkID using prev link ID and current pose

        current_linkID = self.prev_linkID
        self.prev_linkID = current_linkID
        return current_linkID

class MapElementExtractor:

    def __init__(self, cfg):
        self.cfg = cfg

        ngii = self._get_parsed_NGII_map(cfg.map_path)
        self.base_lla = list(map(float, cfg.base_lla))
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

        for target_element in element_names:
            
            if target_element == "HELP":
                print("Available elements: LINEMARK, TRAFFIC SIGN")

            elif target_element == "LINEMARK":

                linkID_of_interest = self._get_id_of_interest(current_linkID)
                points_dict = self._extract_points(target_element, linkID_of_interest)

                tmp_points = None
                for points in points_dict.values():

                    if tmp_points is not None:
                        tmp_points = np.hstack([tmp_points, points])
                    else: ## First time
                        tmp_points = points

                elems_WC_dict[target_element] = np.vstack([tmp_points, np.ones(tmp_points.shape[1])])
        
            elif target_element == "TRAFFIC SIGN":
                # elems_WC_dict[target_element] = self.__extract_traffic_sign(local_map)
                pass

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
        return elems_WC_dict


