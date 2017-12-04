Benjamin A. Slack
CS5260 - Project
Threaded Skin Cluster
An Introduction to the OpenMaya API

Description:
Implements a threaded version of Linear 
Blend Skinning as a Maya plugin node. Uses 
Maya's Intel Threading Building Blocks wrapper 
to allocate and manage the thread pool.

Project files created in Microsoft Visual 
Studio, C++, Community, 2017

Installation:
Copy the threadedSkinCluster.mll into your 
Maya plugin path. 

Usage:
Use the bindThreadedSkinCluster.mel
script to attach joints to your target mesh. 
The plugin extends the MPxSkinCluster proxy 
class, to all standard Maya skin cluster tools 
will be accessible to the user.