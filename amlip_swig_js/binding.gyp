{
  "targets": [
    {
      "target_name": "amlip_swig_js",
      "sources": [
            "../../../build/amlip_swig_js/src/swig/CMakeFiles/amlip_swig_js.dir/amlip_swig_jsJAVASCRIPT_wrap.cxx"
      ],
      "include_dirs" : [ "<!(node -e \"require('nan')\")",
                        '../../../install/amlip_cpp/include',
                        '../../../install/ddsrouter_core/include',
                        '../../../install/ddspipe_participants/include',
                        '../../../install/ddspipe_core/include',
                        '../../../install/cpp_utils/include',
                        '../../../install/fastrtps/include',
                        '../../../install/fastcdr/include',
                        '../../../install/'],
      "libraries": [
            "<(module_root_dir)/../../../build/amlip_cpp/libamlip_cpp.so.0.1.0",
            "<(module_root_dir)/../../../build/ddsrouter_core/libddsrouter_core.so.2.0.0",
            "<(module_root_dir)/../../../build/ddspipe_participants/libddspipe_participants.so.0.2.0",
            "<(module_root_dir)/../../../build/ddspipe_core/libddspipe_core.so.0.2.0",
            "<(module_root_dir)/../../../build/cpp_utils/libcpp_utils.so.0.4.0",
          ],
      "cflags!": [ "-fno-exceptions" ],
      "cflags_cc!": [ "-fno-exceptions" ],
      "cflags_cc": [ "-frtti"],
    }
  ]
}
