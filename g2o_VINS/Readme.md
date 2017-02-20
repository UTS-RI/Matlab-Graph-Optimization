This is the latest version of matlab g2o code.
The framework is reorganized with more necessary warnings for the extension of the new node and new edge.
This code consists of 5 folders:
(1)	Data: store the data to be processed
(2)	Factor: the edge and node 
(3) g2o_files: core, provide the main framework of the least squares
(4) Math: provide the mathematical operation like exp(¡­) ¡­
(5) Auxilliary: others

When the new node is defined, the information needs to be given in
the ¡°GetNodeTypeDimension¡±, ¡°SetNodeDefaultValue¡± and ¡°update_state¡±.

When the new edge is defined, the information needs to be given in
¡°GetFactorX_format¡± and ¡°GetEdgeTypeDimension¡±.

When you want to perform the estimation for 2D RGBD case, just run ¡°Example_VictoriaPark.m¡±.
When you want to perform the estimation for 3D vision case, just run ¡°Vision_Example_Small.m¡±.
