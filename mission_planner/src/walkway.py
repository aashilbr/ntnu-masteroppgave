import pymesh

huldra_models_path = '../../resources/huldra-models/'

mesh = pymesh.load_mesh(huldra_models_path + 'huldra-walkway/meshes/Huldra-small-area-walkway.obj')

print(mesh.num_vertices, mesh.num_faces, mesh.num_voxels)