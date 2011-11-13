FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/draw_circle/msg"
  "../src/draw_circle/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/test-results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test-results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
