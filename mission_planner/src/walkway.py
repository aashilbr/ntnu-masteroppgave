import pymesh

def obj_to_gazebo_coordinates(coordinates):
    a = coordinates[0]
    b = coordinates[1]
    c = coordinates[2]

    x = a
    y = -c
    z = b

    x = x + 116
    y = y + 287
    z = z - 27
    return [x, y, z]

def obj_to_gazebo_coordinates_only_roll(coordinates):
    a = coordinates[0]
    b = coordinates[1]
    c = coordinates[2]

    x = a
    y = -c
    z = b
    return [x, y, z]

huldra_models_path = '../../resources/huldra-models/'

mesh = pymesh.load_mesh(huldra_models_path + 'huldra-smaller-walkway/meshes/huldra-smaller-walkway.obj')

print('Face indices')
mesh.add_attribute('face_index')
face_indices = mesh.get_face_attribute('face_index')
print(face_indices)
print()

print('Face normals')
mesh.add_attribute('face_normal')
face_normals = mesh.get_face_attribute('face_normal')
print(face_normals)
print()

print('Face normals, converted to gazebo coordinates')
face_normals_converted = [0] * len(face_normals)
for i in range(0, len(face_normals)):
    face_normals_converted[i] = obj_to_gazebo_coordinates_only_roll(face_normals[i])
    print(face_normals_converted[i])

print('Indices of faces where z-direction (gc) is positive')
faces_upwards_indices = []
for i in range(0, len(face_normals_converted)):
    if face_normals_converted[i][2] > 0:
        faces_upwards_indices.append(i)
print(faces_upwards_indices)
print()

print('Face centroids')
mesh.add_attribute('face_centroid')
face_centroids = mesh.get_face_attribute('face_centroid')
print(face_centroids)
print()

print('Face centroids, converted to gazebo coordinates')
face_centroids_converted = [0] * len(face_centroids)
for i in range(0, len(face_centroids)):
    face_centroids_converted[i] = obj_to_gazebo_coordinates(face_centroids[i])
    print(face_centroids_converted[i])
print()

print('Face centroids (gc) of upwards faces')
face_centroids_upwards = [0] * len(faces_upwards_indices)
for i, index in enumerate(faces_upwards_indices):
    face_centroids_upwards[i] = face_centroids_converted[index]
    print(face_centroids_upwards[i])
print()