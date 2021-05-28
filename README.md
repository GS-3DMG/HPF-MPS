# HPF-MPS

## Background
Multiple-point geostatistical (MPS) simulation methods have attracted an enormous amount of attention in earth and environmental sciences due to their ability to enhance extraction and synthesis of heterogeneous patterns. To characterize the subtle features of heterogeneous structures and phenomena, large-scale and high-resolution simulations are often required. Accordingly, the size of simulation grids has increased dramatically. Since MPS is a sequential process for each grid unit along a simulation path, it results in severe computational consumption. 

In this work, a new hybrid parallel framework is proposed for the case of MPS simulation on large areas with enormous amount of grid cells. Both inter-node-level and intra-node-level parallel strategies are combined in this framework. To maintain the quality of the realizations, we implement a conflict control method adapting to the Monte-Carlo process. Also, an optimization method for the simulation information is embedded to reduce the inter-node communication overhead. 

## Compiler Environment
This code has been tested in the Unix system:

* MPICC/3.2.1
* GCC/7.2.0

## Examples
A small-scale test data is provided

* Training image: Strbelle_250x250.sgems
* Conditioning data: Con_100.sgems

## Usage
* Complie

mpic++ -o test DS_Simultion.cpp Simulation.h Simulation.cpp -fopenmp

* Execute

mpirun -n -p training ./test

## Contributors

Please report the bugs and error to us. 

We will be happy to remove and enhance the codes.

Thanks,

Email: czs@cug.edu.cn & qiyu.chen@cug.edu.cn

## Maintainer

czs@cug.edu.cn

## License
[MIT]https://github.com/GS-3DMG/HPF-MPS/blob/main/LICENSE
