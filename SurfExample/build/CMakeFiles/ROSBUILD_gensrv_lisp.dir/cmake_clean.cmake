FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/draw_circle/msg"
  "../src/draw_circle/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/AddTwoInts.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AddTwoInts.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
