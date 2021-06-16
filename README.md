# 3D-Reconstructrion-Scanner
**Author: Esteban Andrade**

This will be the developed framework software for a 3D software.
This framework will use phogrammetric software to produce the reconstruction and a series of algorithms that are used in this framework.

The framework will be used for processing, filtering, scaling and reconstructing the final mesh obtained from the photogrammetric process.

## Meshroom
Meshroom is the photogrammetric software used for 3D reconstruction. In this software a series of images will be loaded and it will reconstruct the required Scenes.
It is compatible with Linux and Windows.

Both platforms worked for this project of the 3D scanner. Both Version 2020 and 2021 were used and the results were consistent between each other using the same settings.

### Requirements
NVIDIA CUDA-enabled GPU (built with cuda-10 compatible with compute capability 3.0 to 7.5)

#### Installation
Meshroom can be installed with the following links.

- [GitHub Repo](https://github.com/alicevision/meshroom).
- [Alice Vision](https://alicevision.org/#meshroom)

### How to use
The respective settings and best configuration for each node can be referenced on the Thesis document [Documentation](https://github.com/esteban-andrade/3D-Reconstructrion-Scanner/blob/main/A21%20-%2004017%20Final%20Report%20Esteban%20Andrade%20Zambrano.pdf).

In the above documents there will be documentation of what settings are required for each node. (Refer to section 2.2 on the [Documentation](https://github.com/esteban-andrade/3D-Reconstructrion-Scanner/blob/main/A21%20-%2004017%20Final%20Report%20Esteban%20Andrade%20Zambrano.pdf)).

Once the photogrammetric process has finished the mesh can be retrieved from either the *meshing* or *filter mesh node* as .obj file  from the cache folder from meshroom.

For best result, as these meshes will be used in the *Reconstruction Framework*, convert the meshes from *.obj* to *.ply* using **[Meshlab](https://www.meshlab.net/)**.

## Reconstruction Framework.
The reconstruction Framework is a series of classes and program that was develop in order to process, scale and recosstruct the Mesh result from Meshroom.

### Requirements
This framework was developed in Linux. However windows Functionality was added in order to build  and compile the files.

The requirements include

- PCL Library (1.2 Minimum) (1.8 Ideal)
- Open3D
- Matlab 
- OpenCV (3.9.2 Minimum )
- Boost
- C++ (14 Minimum)
- Python (3.6.9 Minimum)
- Ninja Compiler (Optional).

### Installation
In order to install this framework its neccesary to build and compile this project. 

##### Standard Method

```
git clone https://github.com/esteban-andrade/3D-Reconstructrion-Scanner
mkdir build
cd build
cmake ..
make
```
##### Prefered Method
The prefered method is using Ninja Compiler as it faster than standard gcc

```
git clone https://github.com/esteban-andrade/3D-Reconstructrion-Scanner
mkdir build
cd build
cmake -GNinja ..
ninja
```

### Execution

In order to execute the framework it is neccesary to have two files *(.ply files)*
The first file has to be the target file that needs to be processed *(Output from meshrroom*. The second file has to be the file that will be used for scaling*(Lidar)*

The first term is the path to the reference target file. The second term is the path to the scale reference file.
The last term is a command to generate output files. (Recommended to set to yes). All the output saved on the *Meshes Folder*

##### In order to execute:
```
cd build 
./3D_reconstruction_pipeline [Path_to_referenceFile] [Path_to_SCaling_file] [yes/no](Generate output files)
```

##### Example
```
./3D_reconstruction_pipeline ../Data/mesh_ply.ply ../Data/10.ply no
```

#### Expected Result
The expeted result will output a viewer with all the steps of the pipeline processing.
If There is the need to change parameters, change then at the top of the *Reconstruction.cpp* file. 
To change the filtering adjust as needed on *main.cpp*. 
Ideally the default settings will work with most datasets.

##### Poisson Reconstruction Output.
In certain occasions when the poisson reconstruction steps fails due to wrong normals direction.
Possion script [Possion Open3D](https://github.com/esteban-andrade/3D-Reconstructrion-Scanner/blob/main/poisson_open3d.py)

This Script will require to use the output files from the process. (Ensure executable (*3D_reconstruction_pipeline*) was set to **YES**)
This script will adjust the normals direction and run Poisson again.

To execute:

```
python3 poisson_open3d.py
```

### Extra components
An Extra component added  and adapted to this framework is [SCaleRatioICP](https://github.com/linbaowei/ScaleRatioICP)
The matlab components  of this class were added here under [ScaleRatioICP](https://github.com/esteban-andrade/3D-Reconstructrion-Scanner)
In order to use this object ensure that **ScaleRatioICP.m** is added to the matlab path.

All the framework implementation was fully integrated and adapted. 

### Documentation
To visualise the documentation 

```
firefox html/index.html 
```
