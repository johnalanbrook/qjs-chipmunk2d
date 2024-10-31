project('qjs-chipmunk', 'c')

quickjs = dependency('quickjs', static:true)
chipmunk = dependency('chipmunk', static:true)

qjs_chipmunk_lib = library('qjs-chipmunk',
  'qjs_chipmunk.c',
  dependencies: [quickjs, chipmunk],
  c_shared_args: '-DJS_SHARED_LIBRARY',    
)

qjs_chipmunk_dep = declare_dependency(
  link_with:qjs_chipmunk_lib,
  dependencies: [chipmunk],
)