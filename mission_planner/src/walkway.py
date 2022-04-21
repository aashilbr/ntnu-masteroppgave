import pymesh
from utils import *

class Walkway():
    def __init__(
        self, 
        models_path      = '../../resources/huldra-models/',
        walkway_path     = 'huldra-smaller-walkway/meshes/',
        walkway_filename = 'huldra-smaller-walkway.obj',
        ):

        walkway_file = models_path + walkway_path + walkway_filename

        self.walkway_line = self._find_walkway_line(walkway_file)

    def _find_walkway_line(self, walkway_file):
        walkway_mesh = pymesh.load_mesh(walkway_file)

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

        # TODO: Return the list sorted, from one end of the walkway to the other end
        #       or return the list so that the line is as short as possible (this might solve the sorting problem)

        points = [None] * len(face_centroids_upwards)
        for i, centroid in enumerate(face_centroids_upwards):
            points[i] = Point(centroid[0], centroid[1], centroid[2])
        
        return points
    
    def get_walkway_line(self):
        return self.walkway_line
    
    def get_points_along_walkway_with_resolution(self, resolution):
        # Note: "resolution" means distance between the returned points.
        #       This means that (counter intuitively, I know) that e.g. resolution=10 actually means a lower resolution than resolution=1.
        
        resolution_points = []
        resolution_points.append(self.walkway_line[0])

        unused_distance_previous_iteration = 0
        for i in range(0, len(self.walkway_line) - 1):
            p0 = self.walkway_line[i]
            p1 = self.walkway_line[i + 1]

            distance_p0_p1 = get_distance_between_points(p0, p1)
            distance_walked_from_p0 = 0
            next_distance_to_walk = resolution - unused_distance_previous_iteration

            j = 1
            while distance_p0_p1 - (distance_walked_from_p0 + next_distance_to_walk) > 0:
                resolution_points.append(get_point_between_at_distance(p0, p1, distance_walked_from_p0 + next_distance_to_walk))
                distance_walked_from_p0 += next_distance_to_walk

                next_distance_to_walk = resolution
                unused_distance_previous_iteration = 0
                j += 1
            unused_distance_previous_iteration = unused_distance_previous_iteration + distance_p0_p1 - distance_walked_from_p0
        
        resolution_points.append(self.walkway_line[-1])

        return resolution_points