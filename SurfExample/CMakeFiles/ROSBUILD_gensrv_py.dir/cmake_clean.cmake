FILE(REMOVE_RECURSE
  "src/draw_circle/msg"
  "src/draw_circle/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/draw_circle/srv/__init__.py"
  "src/draw_circle/srv/_AddTwoInts.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
