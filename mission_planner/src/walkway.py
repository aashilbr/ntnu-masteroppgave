from ast import walk
import pymesh
from utils import *

class WalkwayProcessor():
    def __init__(
        self, 
        huldra_models_path = '../../resources/huldra-models/',
        huldra_walkway_path = 'huldra-smaller-walkway/meshes/',
        huldra_walkway_filename = 'huldra-smaller-walkway.obj',
        ):
        self.huldra_models_path = huldra_models_path
        self.huldra_walkway_path = huldra_walkway_path
        self.huldra_walkway_filename = huldra_walkway_filename

        self.walkway_line = self._find_walkway_line()

    def _find_walkway_line(self):
        walkway_mesh = pymesh.load_mesh(self.huldra_models_path + self.huldra_walkway_path + self.huldra_walkway_filename)

        walkway_mesh.add_attribute('face_normal')
        face_normals = walkway_mesh.get_face_attribute('face_normal')

        face_normals_converted = [0] * len(face_normals)
        for i in range(0, len(face_normals)):
            face_normals_converted[i] = obj_to_gazebo_coordinates_only_roll(face_normals[i])

        faces_upwards_indices = []
        for i in range(0, len(face_normals_converted)):
            if face_normals_converted[i][2] > 0:
                faces_upwards_indices.append(i)

        walkway_mesh.add_attribute('face_centroid')
        face_centroids = walkway_mesh.get_face_attribute('face_centroid')

        face_centroids_converted = [0] * len(face_centroids)
        for i in range(0, len(face_centroids)):
            face_centroids_converted[i] = obj_to_gazebo_coordinates(face_centroids[i])

        face_centroids_upwards = [0] * len(faces_upwards_indices)
        for i, index in enumerate(faces_upwards_indices):
            face_centroids_upwards[i] = face_centroids_converted[index]

        # TODO: Find end points on walkway and add them to the returned list
        
        return face_centroids_upwards
    
    def get_walkway_line(self):
        return self.walkway_line
    
    def get_points_along_walkway_with_resolution(self, resolution):
        if resolution == 0:
            return self.walkway_line
        else:
            points = []
            points.append(self.walkway_line[0])
            points.append(self.walkway_line[-1])

            #for i in len(0, len(walkway_line) - 1):
            #    distance_to_next_point = get_distance_between_points(walkway_line[i], walkway_line[i + 1])
        
        return points