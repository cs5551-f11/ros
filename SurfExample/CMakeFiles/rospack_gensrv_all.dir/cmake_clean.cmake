FILE(REMOVE_RECURSE
  "src/draw_circle/msg"
  "src/draw_circle/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/rospack_gensrv_all"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_gensrv_all.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
