project('chipmunk2d', 'c')

quickjs = dependency('quickjs', fallback:['quickjs', 'quickjs_dep'])
chipmunk2d = dependency('chipmunk2d', fallback:['Chipmunk2D', 'chipmunk2d_dep'])

libtype = get_option('default_library')
if libtype == 'static'
  add_project_arguments('-DJS_SHARED_LIBRARY', language:'c')
endif

qjs_chipmunk_lib = library('qjs-chipmunk2d',
  'qjs_chipmunk2d.c',
  dependencies: [quickjs, chipmunk2d]
)
