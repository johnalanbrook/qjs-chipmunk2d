project('qjs-chipmunk', 'c')

quickjs = dependency('quickjs', static:true)
chipmunk = dependency('chipmunk', static:true)

libtype = get_option('default_library')
if libtype == 'static'
  add_project_arguments('-DJS_SHARED_LIBRARY', language:'c')
endif

qjs_chipmunk_lib = library('qjs-chipmunk',
  'qjs_chipmunk.c',
  dependencies: [quickjs, chipmunk]
)

qjs_chipmunk_dep = declare_dependency(
  link_with:qjs_chipmunk_lib,
  dependencies: [chipmunk],
)