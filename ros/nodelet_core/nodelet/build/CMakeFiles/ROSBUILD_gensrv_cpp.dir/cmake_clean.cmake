FILE(REMOVE_RECURSE
  "../src/nodelet/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/nodelet/NodeletLoad.h"
  "../srv_gen/cpp/include/nodelet/NodeletList.h"
  "../srv_gen/cpp/include/nodelet/NodeletUnload.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
