FILE(REMOVE_RECURSE
  "../src/nodelet/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/NodeletLoad.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_NodeletLoad.lisp"
  "../srv_gen/lisp/NodeletList.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_NodeletList.lisp"
  "../srv_gen/lisp/NodeletUnload.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_NodeletUnload.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
