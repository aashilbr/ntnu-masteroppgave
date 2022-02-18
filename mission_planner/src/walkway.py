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

    def find_walkway_line(self):
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
        
        return face_centroids_upwards

if __name__ == '__main__':
    wp = WalkwayProcessor()
    walkway_line = wp.find_walkway_line()
    print('Walkway line:')
    print(walkway_line)
    publish_markers(walkway_line)

# Find points on walkway

# Check distance

# Check raycast collisions

# (Check if we can see the front of the valve)

# -> send to image analysis

# Sort points on best weighted result