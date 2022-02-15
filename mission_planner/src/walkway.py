import pymesh

huldra_models_path = '../../resources/huldra-models/'

mesh = pymesh.load_mesh(huldra_models_path + 'huldra-smaller-walkway/meshes/huldra-smaller-walkway.obj')

mesh.add_attribute('face_index')
print(mesh.get_face_attribute('face_index'))

mesh.add_attribute('face_normal')
print(mesh.get_face_attribute('face_normal'))