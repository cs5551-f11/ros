FILE(REMOVE_RECURSE
  "src/draw_circle/msg"
  "src/draw_circle/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/test-future"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test-future.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)