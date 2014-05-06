FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/joost/msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/src/Arm.cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
