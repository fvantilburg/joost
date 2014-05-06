FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/joost/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/joost/JoltJoint.h"
  "../msg_gen/cpp/include/joost/JoltArmStatus.h"
  "../msg_gen/cpp/include/joost/Jolt4DOF.h"
  "../msg_gen/cpp/include/joost/MotorStatus.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
