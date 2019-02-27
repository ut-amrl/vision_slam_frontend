file(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/vision_slam_frontend/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rospack_genmsg_libexe.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
