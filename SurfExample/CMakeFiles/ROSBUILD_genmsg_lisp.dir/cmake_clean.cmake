FILE(REMOVE_RECURSE
  "src/SurfExample/msg"
  "src/SurfExample/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/Num.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Num.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
