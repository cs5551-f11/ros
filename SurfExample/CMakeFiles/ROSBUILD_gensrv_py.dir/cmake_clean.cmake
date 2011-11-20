FILE(REMOVE_RECURSE
  "src/SurfExample/msg"
  "src/SurfExample/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/SurfExample/srv/__init__.py"
  "src/SurfExample/srv/_AddTwoInts.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
