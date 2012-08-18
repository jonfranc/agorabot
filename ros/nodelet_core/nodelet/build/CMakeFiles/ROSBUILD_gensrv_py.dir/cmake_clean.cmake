FILE(REMOVE_RECURSE
  "../src/nodelet/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/nodelet/srv/__init__.py"
  "../src/nodelet/srv/_NodeletLoad.py"
  "../src/nodelet/srv/_NodeletList.py"
  "../src/nodelet/srv/_NodeletUnload.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
