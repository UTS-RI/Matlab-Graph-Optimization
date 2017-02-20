Author: Teng Zhang
This code is the general nonlinear least squares optimization for SLAM, that follows the well-known optimization C++ code packages g2o and gtsam styles.

The framework is organized with some necessary warnings for the extension of the new node and new edge.
Therefore, user can easily define some specific Node/Vertex and Factor/Edge by using this code. 

When defining a new factor, the compute error and Jacobians need to given in the "Factor" folder.
When the new node is defined, the information needs to be given in the “GetNodeTypeDimension”, “SetNodeDefaultValue” and “update_state”.
When the new edge is defined, the information needs to be given in “GetFactorX_format” and “GetEdgeTypeDimension”.
Now this code includes some common Factors like IMUfactor, VisionFactor and Nodes like Pose3 and Landmark3.

Example_VictoriaPark.m may be a good option for initial understanding.
