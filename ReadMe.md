# Interactive Flow Fields Visualization based on Gesture Query in 3-D with Leap Motion

This is the project for my undergraduate thesis (2020.06).

The paper is also accepted by IEEE VIS 2020 as a poster paper [1].

Note that we only uploaded the core src files here.

So the project **cannot be successfully installed** with these files.

# Introduction
Flow field visualization is an important topic in the field of Scientific Visualization.
They mainly focus on the simulation of vector field features using geometric elements such as lines, surfaces, and bodies.
Related research topics include computational efficiency, visualization of complex meshes and multiple variables, feature extraction and tracking such as uncertainty, etc.
Specifically, This research is mainly about feature extraction.

Except for statistical features, user-defined derived features are also a vital target of flow field visualization.
Interactive visualization of such features would introduce domain knowledge of natural science experts into the analysis process.
Nevertheless, traditional methods including sketching (with the mouse) or querying (by Domain Specific Language) enforce limits the exploration into 2-D environments,
though the flow field data could be 3-D in most cases.

We design this system so as to support interactive exploration of 3-D flow field data visualization for natural science experts
with the help of [Leap Motion](https://en.wikipedia.org/wiki/Leap_Motion), supporting figures and hands movement as inputs (without the mouse, hand contact, or touching),
so as to further support flow field data analysis and decision making during the visualization exploration.

## The overview of this system
![](https://gyazo.com/2af31f0cc3671ab6dcfba6f003b80263.png)

## The pipeline of this system
![](https://gyazo.com/a1929e3e07754190a99dacb0635bece1.png)

# Dependencies
The system is realized based on DTI Fiber Explorer[2], with C++ (MFC and OpenGL).

IDE: Microsoft Visual Studio 2019.

SDK: Leap Motion SDK 4.0.0.

# To Use
1. Select the dataset to load.
2. Input the user-defined derived feature for querying.
   - You can both input it in real-time, or instead of loading an existing 2-D or 3-D curve file as input.
   - While inputting it in real-time, both mouse or leap motion are available.
   - Notice that the input curve could be saved in local, so that you can load it next time.
3. You can interactively drag, rotate or zoom the main view so as to explore the resulted visualization.
4. Parameter adjustment is always available during the exploration. The result would be updated immediately.

# User Cases
Notice that the green curve is 2-D input with mouse (sketching), while the blue curve is 3-D input with Leap Motion.

## Dataset 1: Trajectories of Water Molecules Movement in the Heart
![](https://gyazo.com/f85695b41d827406c3aa81a67e2d813f.png)

## Dataset 2: Trajectories of Water Molecules Movement in the Brain
![](https://gyazo.com/bdbe632d063edd662b9028faf295829f.png)

## Dataset 3 : GEOS-5 Flow Field Ensemble Simulations
![](https://gyazo.com/e2c52ccc6eeb97c42aef367fc46521c8.png)

# Reference
[1] Shunlong Ye†, Guang Yang†, Ziyu Yao†, Xueyi Chen†, Ting Jin†, Genlin Ji, and Richen Liu*.
Robust 3-D Field Line Query Based on Data Fusion of Multiple Leap Motions. IEEE Visualization 2020 (IEEE VIS), Salt Lake City, USA, October 25-30, 2020. (CCF A Poster).

[2] W. Chen, Z. Ding, S. Zhang, A. MacKay-Brandt, S. Correia, H. Qu, J. A. Crow, D. F. Tate, Z. Yan, and Q. Peng. 
Open source codes of dti fiber explorer. https://sourceforge.net/, 2009. A Novel Interface for Interactive Exploration of DTI Fibers.

# Appendix
- Detailed slides could be accessed [here](https://www.dropbox.com/s/gahzhnwnt3ivxxf/FlowFieldVisWithLeapMotion.pdf?dl=0).
