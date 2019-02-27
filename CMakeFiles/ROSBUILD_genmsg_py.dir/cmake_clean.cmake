file(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/vision_slam_frontend/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/vision_slam_frontend/msg/__init__.py"
  "src/vision_slam_frontend/msg/_VisionFactor.py"
  "src/vision_slam_frontend/msg/_RobotPose.py"
  "src/vision_slam_frontend/msg/_FeatureMatch.py"
  "src/vision_slam_frontend/msg/_OdometryFactor.py"
  "src/vision_slam_frontend/msg/_SLAMProblem.py"
  "src/vision_slam_frontend/msg/_SLAMNode.py"
  "src/vision_slam_frontend/msg/_VisionFeature.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
