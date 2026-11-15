# Color point cloud direct alignment to equirectangular image
 
## CNRS-AIST JRL, IRL

## Author: Guillaume Caron

## Dates: September 2024

0. Prerequisites to run these codes: 
     - install cmake (version 3.16.3 tested)
     - install ceres-solver (version 2.0.0, github, master, commit 8d3e64dd5e64b346ed3e412cba2b70d760881bfb tested)
     - install opencv (version 4.2.0 tested)
     - download test data from: http://mis.u-picardie.fr/~g-caron/data/2024_rgbd2rgbalign_media.zip and set its content in the 2024_rgbd2rgbalign_media directory at the same level as the rgbd2rgbalign directory
1. create a new directory named rgbd2rgbalign-build at the same level as the rgbd2rgbalign directory
2. use cmake to fill the rgbd2rgbalign-build directory in, with variables: `cmake ../rgbd2rgbalign -DCMAKE_BUILD_TYPE=Release`
3. open the project in rgbd2rgbalign-build or use the make command in the latter directory to build the exe file
4. run the program on the command line from the rgbd2rgbalign-build directory with arguments as:
     ```
     ./align_pc2equi \
          -desired ../2024_rgbd2rgbalign_media/frame0000/equi_frame0000.jpg \
          -input ../2024_rgbd2rgbalign_media/frame0000/point_cloud_fra0_u.pts \
          -output ../2024_rgbd2rgbalign_media/frame0000/pc2equi_output0000.jpg \
          -overlayedOutput ../2024_rgbd2rgbalign_media/frame0000/pc2equi_overlay0000.jpg
     ```

## Paper to cite

G. Caron, G. Sevillano, M. Paula, N. Imamoglu. "RGB-D 360-degree SLAM with Learned Depth". IEEE/SICE International Symposium on System Integration, Jan. 2027, Kobe, Japan. 
