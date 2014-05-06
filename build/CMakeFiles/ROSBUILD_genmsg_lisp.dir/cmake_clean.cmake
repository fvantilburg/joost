FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/joost/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/JoltJoint.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_JoltJoint.lisp"
  "../msg_gen/lisp/JoltArmStatus.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_JoltArmStatus.lisp"
  "../msg_gen/lisp/Jolt4DOF.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Jolt4DOF.lisp"
  "../msg_gen/lisp/MotorStatus.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MotorStatus.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
