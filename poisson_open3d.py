import open3d as o3d
import numpy as np
import copy
import matplotlib.pyplot as plt


print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("Meshes/smoothed_cloud_refined.ply")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

saver = False

if saver== True:
    print("\n.................Generating Files..............\n")

pcd.normals = o3d.utility.Vector3dVector(np.zeros(
    (1, 3)))  # invalidate existing normals

pcd.estimate_normals()
#o3d.visualization.draw_geometries([pcd], point_show_normal=True)

# print('run Poisson surface reconstruction')
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=9)
# print(mesh)
# o3d.visualization.draw_geometries([mesh])

pcd.orient_normals_consistent_tangent_plane(10) #10
#o3d.visualization.draw_geometries([pcd], point_show_normal=True)


print('run After normal Estimation Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=7)
print(mesh)
o3d.visualization.draw_geometries([mesh])
if saver==True:
    o3d.io.write_triangle_mesh("Meshes/poisson_open3d.ply", mesh)


print('visualize densities')
densities = np.asarray(densities)
density_colors = plt.get_cmap('plasma')(
    (densities - densities.min()) / (densities.max() - densities.min()))
density_colors = density_colors[:, :3]
density_mesh = o3d.geometry.TriangleMesh()
density_mesh.vertices = mesh.vertices
density_mesh.triangles = mesh.triangles
density_mesh.triangle_normals = mesh.triangle_normals
density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([density_mesh])

print('remove low density vertices')
vertices_to_remove = densities < np.quantile(densities, 0.025)  #0.01
mesh.remove_vertices_by_mask(vertices_to_remove)
print(mesh)
o3d.visualization.draw_geometries([mesh])
if saver == True:
    o3d.io.write_triangle_mesh("Meshes/poisson_open3d_after_removal.ply", mesh)



#http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html


print("\nLoad a ply point cloud No Upscaling, print it, and render it")
pcd2 = o3d.io.read_point_cloud("Meshes/target_cloud_scaled_aligned.ply")
print(pcd2)
print(np.asarray(pcd2.points))
o3d.visualization.draw_geometries([pcd2])


pcd2.normals = o3d.utility.Vector3dVector(np.zeros(
    (1, 3)))  # invalidate existing normals

pcd2.estimate_normals()
#o3d.visualization.draw_geometries([pcd], point_show_normal=True)

# print('run Poisson surface reconstruction')
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=9)
# print(mesh)
# o3d.visualization.draw_geometries([mesh])

pcd2.orient_normals_consistent_tangent_plane(10)  # 10
#o3d.visualization.draw_geometries([pcd], point_show_normal=True)


print('run After normal Estimation Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh2, densities2 = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd2, depth=7)
print(mesh2)
o3d.visualization.draw_geometries([mesh2])
if saver == True:
    o3d.io.write_triangle_mesh(
    "Meshes/poisson_open3d_target_cloud_scaled_aligned.ply", mesh2)


print('visualize densities')
densities2 = np.asarray(densities2)
density_colors2 = plt.get_cmap('plasma')(
    (densities2 - densities2.min()) / (densities2.max() - densities2.min()))
density_colors2 = density_colors2[:, :3]
density_mesh2 = o3d.geometry.TriangleMesh()
density_mesh2.vertices = mesh2.vertices
density_mesh2.triangles = mesh2.triangles
density_mesh2.triangle_normals = mesh2.triangle_normals
density_mesh2.vertex_colors = o3d.utility.Vector3dVector(density_colors2)
o3d.visualization.draw_geometries([density_mesh2])

print('remove low density vertices')
vertices_to_remove2 = densities2 < np.quantile(densities2, 0.025)  # 0.01
mesh2.remove_vertices_by_mask(vertices_to_remove2)
print(mesh2)
o3d.visualization.draw_geometries([mesh2])
if saver == True:
    o3d.io.write_triangle_mesh(
    "Meshes/poisson_open3d_after_removal_target_cloud_scaled_aligned.ply", mesh2)
