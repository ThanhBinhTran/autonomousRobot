from collections import defaultdict

from Robot_math_lib import point_dist


class Sight:
    def __init__(self):
        # explored sight
        self.closed_sights = defaultdict(list)  
        self.open_sights = defaultdict(list)  
    
    ''' add a neighbor sight into explored sight'''
    def add_sight(self, center, closed_sights, open_sights):
        self.closed_sights[center] = closed_sights
        self.open_sights[center] = open_sights

    ''' get sight by coordinate '''
    def get_sight(self, center):
        csights = self.closed_sights[center]
        ocights = self.open_sights[center]
        return csights, ocights

    ''' get sight by coordinate '''
    def get_closed_sights(self, center):
        csights = self.closed_sights[center]
        return csights

    ''' get sight by coordinate '''
    def get_open_sights(self, center):
        cpt = center[0], center[1]      # ndarray is unhashable, then covert to tuple
        ocights = self.open_sights[cpt]
        return ocights   
     
    ''' get size of sights '''
    def size(self):
        return len(self.closed_sights)
    
    ''' get all coordinate '''
    def all_coordinate(self):
        all_keys = list(self.closed_sights.keys())
        return all_keys
    
    ''' get neighbor coordinate in range d from a given point '''
    def get_neighbors_inrange(self, point, range_d):
        neighbors = []
        all_coords = self.all_coordinate()
        for center in all_coords:
            if point_dist(center, point) < range_d:
                neighbors.append(center)
        return neighbors
