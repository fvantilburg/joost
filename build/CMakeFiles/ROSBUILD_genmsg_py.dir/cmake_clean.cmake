FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/joost/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/joost/msg/__init__.py"
  "../src/joost/msg/_JoltJoint.py"
  "../src/joost/msg/_JoltArmStatus.py"
  "../src/joost/msg/_Jolt4DOF.py"
  "../src/joost/msg/_MotorStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
