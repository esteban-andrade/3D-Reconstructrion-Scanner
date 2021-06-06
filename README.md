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

Once the photogrammetric process has finished the mesh can be retrieved from either the *meshing* or *filter mesh node*

## Reconstruction Framework.



