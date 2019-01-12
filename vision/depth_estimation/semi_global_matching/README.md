# Semi-global matching for depth estimation

The code is adapted from [libSGM](https://github.com/fixstars/libSGM). There are two reason for using this "traditional" approach:
* Be able to do depth estimation in high speed. 
* We do not need very accurate depth estimation, as long as it can give us some hints for trajectory estimation.  
* Have seen some use cases in quadcopter.