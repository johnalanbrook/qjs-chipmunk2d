
#include <chipmunk/chipmunk.h>
#include <chipmunk/chipmunk_unsafe.h>
#include <quickjs.h>

#define countof(x) (sizeof(x)/sizeof((x)[0]))

static JSClassID js_cpspace_class_id;
static JSClassID js_cpbody_class_id;
static JSClassID js_cpconstraint_class_id;
static JSClassID js_cpshape_class_id;

#define FNSIG (JSContext *js, JSValueConst this_val, int argc, JSValue *argv)
#define GETSIG (JSContext *js, JSValueConst this_val)
#define SETSIG (JSContext *js, JSValueConst this_val, JSValue val)

#define CPCLASS(TYPE) \
static JSClassID js_##TYPE##_class_id; \
static inline TYPE *js2##TYPE(JSContext *js, JSValue v) { \
  return JS_GetOpaque(v, js_##TYPE##_class_id); \
} \
static inline JSValue TYPE##2js(JSContext *js, TYPE *cp) { \
  return JS_DupValue(js, *(JSValue*)TYPE##GetUserData(cp)); \
} \

CPCLASS(cpBody)
CPCLASS(cpSpace)
CPCLASS(cpConstraint)
CPCLASS(cpShape)

static JSValue js_gear;
static JSValue js_pin;
static JSValue js_pivot;
static JSValue js_rotary;
static JSValue js_motor;
static JSValue js_damped_spring;
static JSValue js_damped_rotary;
static JSValue js_groove;
static JSValue js_slide;
static JSValue js_ratchet;

static inline cpVect js2cpvect(JSContext *js, JSValue v) {
  cpVect ret;
  JSValue x = JS_GetPropertyStr(js, v, "x");
  JSValue y = JS_GetPropertyStr(js, v, "y");
  JS_ToFloat64(js, &ret.x, x);
  JS_ToFloat64(js, &ret.y, y);
  JS_FreeValue(js, x);
  JS_FreeValue(js, y);
  return ret;
}

static inline JSValue cpvect2js(JSContext *js, cpVect v) {
  JSValue obj = JS_NewObject(js);
  JS_SetPropertyStr(js, obj, "x", JS_NewFloat64(js, v.x));
  JS_SetPropertyStr(js, obj, "y", JS_NewFloat64(js, v.y));
  return obj;
}

static inline JSValue angle2js(JSContext *js, double angle) {
  return JS_NewFloat64(js, angle);
}

static inline double js2angle(JSContext *js, JSValue v) {
  double angle;
  JS_ToFloat64(js, &angle, v);
  return angle;
}

static inline JSValue number2js(JSContext *js, double number) {
  return JS_NewFloat64(js, number);
}

static inline double js2number(JSContext *js, JSValue v) {
  double number;
  JS_ToFloat64(js, &number, v);
  return number;
}

static void js_space_finalizer(JSRuntime *rt, JSValue val) {
  cpSpace *space = JS_GetOpaque(val, js_cpspace_class_id);
  if (space) {
    cpSpaceFree(space);
  }
}

////// CP SPACE ///////

static JSClassDef js_space_class = {
  "Space",
  .finalizer = js_space_finalizer
};

static JSValue js_make_space(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpSpace *space = cpSpaceNew();
  JSValue obj = JS_NewObjectClass(js, js_cpspace_class_id);
  JS_SetOpaque(obj, space);
  return obj;
}

static JSValue js_space_set_gravity SETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  JSValue vec_obj = val;
  cpVect gravity = js2cpvect(js, val);
  cpSpaceSetGravity(space, gravity);
  return JS_UNDEFINED;
}

static JSValue js_space_get_gravity GETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  cpVect gravity = cpSpaceGetGravity(space);
  return cpvect2js(js, gravity);
}

static JSValue js_space_get_iterations GETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  int iterations = cpSpaceGetIterations(space);
  return JS_NewInt32(js, iterations);
}

static JSValue js_space_set_iterations SETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  int iterations;
  if (JS_ToInt32(js, &iterations, val)) {
    return JS_EXCEPTION;
  }
  cpSpaceSetIterations(space, iterations);
  return JS_UNDEFINED;
}

static JSValue js_space_get_idle_speed GETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  cpFloat idle_speed = cpSpaceGetIdleSpeedThreshold(space);
  return JS_NewFloat64(js, idle_speed);
}

static JSValue js_space_set_idle_speed SETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  double idle_speed = js2number(js, val);
  cpSpaceSetIdleSpeedThreshold(space, idle_speed);
  return JS_UNDEFINED;
}

static JSValue js_space_get_sleep_time GETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  cpFloat sleep_time = cpSpaceGetSleepTimeThreshold(space);
  return JS_NewFloat64(js, sleep_time);
}

static JSValue js_space_set_sleep_time SETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  double sleep_time = js2number(js, val);
  cpSpaceSetSleepTimeThreshold(space, sleep_time);
  return JS_UNDEFINED;
}

static JSValue js_space_get_collision_slop GETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  cpFloat collision_slop = cpSpaceGetCollisionSlop(space);
  return JS_NewFloat64(js, collision_slop);
}

static JSValue js_space_set_collision_slop SETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  double collision_slop = js2number(js, val);
  cpSpaceSetCollisionSlop(space, collision_slop);
  return JS_UNDEFINED;
}

static JSValue js_space_get_collision_bias GETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  cpFloat collision_bias = cpSpaceGetCollisionBias(space);
  return JS_NewFloat64(js, collision_bias);
}

static JSValue js_space_set_collision_bias SETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  double collision_bias = js2number(js, val);
  cpSpaceSetCollisionBias(space, collision_bias);
  return JS_UNDEFINED;
}
static JSValue js_space_get_collision_persistence GETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  unsigned int collision_persistence = cpSpaceGetCollisionPersistence(space);
  return JS_NewUint32(js, collision_persistence);
}

static JSValue js_space_set_collision_persistence SETSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  uint32_t collision_persistence;
  if (JS_ToUint32(js, &collision_persistence, val)) {
    return JS_EXCEPTION;
  }
  cpSpaceSetCollisionPersistence(space, collision_persistence);
  return JS_UNDEFINED;
}

static void each_body_callback(cpBody *body, void *data) {
  JSContext *js = (JSContext *)data;
  JSValue callback = *(JSValue *)data;
//  JSValue arg = cpbody2js(js, body); // Assuming cpbody2js converts cpBody to a JS object.

//  JS_Call(js, callback, JS_UNDEFINED, 1, &arg);
//  JS_FreeValue(js, arg);
}

static JSValue js_space_each_body(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpSpace *space = js2cpSpace(js, this_val);

  if (!JS_IsFunction(js, argv[0])) {
    return JS_ThrowTypeError(js, "Expected a function");
  }

  JSValue callback = JS_DupValue(js, argv[0]);
  cpSpaceEachBody(space, each_body_callback, &callback);
  JS_FreeValue(js, callback);

  return JS_UNDEFINED;
}

static void each_shape_callback(cpShape *shape, void *data) {
  JSContext *js = (JSContext *)data;
  JSValue callback = *(JSValue *)data;
//  JSValue arg = cpshape2js(js, shape); // Assuming cpshape2js converts cpShape to a JS object.

//  JS_Call(js, callback, JS_UNDEFINED, 1, &arg);
//  JS_FreeValue(js, arg);
}

static JSValue js_space_each_shape(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpSpace *space = js2cpSpace(js, this_val);

  if (!JS_IsFunction(js, argv[0])) {
    return JS_ThrowTypeError(js, "Expected a function");
  }

  JSValue callback = JS_DupValue(js, argv[0]);
  cpSpaceEachShape(space, each_shape_callback, &callback);
  JS_FreeValue(js, callback);

  return JS_UNDEFINED;
}

static void each_constraint_callback(cpConstraint *constraint, void *data) {
  JSContext *js = (JSContext *)data;
  JSValue callback = *(JSValue *)data;
//  JSValue arg = cpconstraint2js(js, constraint); // Assuming cpconstraint2js converts cpConstraint to a JS object.

//  JS_Call(js, callback, JS_UNDEFINED, 1, &arg);
//  JS_FreeValue(js, arg);
}

static JSValue js_space_each_constraint(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpSpace *space = js2cpSpace(js, this_val);

  if (!JS_IsFunction(js, argv[0])) {
    return JS_ThrowTypeError(js, "Expected a function");
  }

  JSValue callback = JS_DupValue(js, argv[0]);
  cpSpaceEachConstraint(space, each_constraint_callback, &callback);
  JS_FreeValue(js, callback);

  return JS_UNDEFINED;
}

static JSValue js_space_step FNSIG {
  cpSpace *space = js2cpSpace(js, this_val);
  return JS_UNDEFINED;
}

static const JSCFunctionListEntry js_space_funcs[] = {
  JS_CFUNC_DEF("step", 1, js_space_step),
  JS_CGETSET_DEF("gravity", js_space_get_gravity, js_space_set_gravity),
  JS_CGETSET_DEF("iterations", js_space_get_iterations, js_space_set_iterations),
  JS_CFUNC_DEF("eachBody", 1, js_space_each_body),
  JS_CFUNC_DEF("eachShape", 1, js_space_each_shape),
  JS_CFUNC_DEF("eachConstraint", 1, js_space_each_constraint),
  JS_CGETSET_DEF("idle_speed", js_space_get_idle_speed, js_space_set_idle_speed),
  JS_CGETSET_DEF("sleep_time", js_space_get_sleep_time, js_space_set_sleep_time),
  JS_CGETSET_DEF("collision_slop", js_space_get_collision_slop, js_space_set_collision_slop),
  JS_CGETSET_DEF("collision_bias", js_space_get_collision_bias, js_space_set_collision_bias),
  JS_CGETSET_DEF("collision_persistence", js_space_get_collision_persistence, js_space_set_collision_persistence),
};


////// CPBODY ///////

#define JS_GETSET_BODY(NAME, TYPE) \
  static JSValue js_body_set_##NAME SETSIG { \
    cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION; \
    cpBodySet##NAME(body, js2##TYPE(js, val)); \
    return JS_UNDEFINED; \
  } \
  static JSValue js_body_get_##NAME GETSIG { \
    cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION; \
    return TYPE##2js(js, cpBodyGet##NAME(body)); \
  }

JS_GETSET_BODY(Position, cpvect)
JS_GETSET_BODY(Angle, angle)
JS_GETSET_BODY(Velocity, cpvect)
JS_GETSET_BODY(AngularVelocity, angle)
JS_GETSET_BODY(Moment, number)
JS_GETSET_BODY(Torque, number)
JS_GETSET_BODY(Mass, number)
JS_GETSET_BODY(CenterOfGravity, cpvect)
JS_GETSET_BODY(Force, cpvect)
JS_GETSET_BODY(Type, number)

static JSValue js_body_apply_force_at_world_point(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpVect force = js2cpvect(js, argv[0]);
  cpVect point = js2cpvect(js, argv[1]);
  cpBodyApplyForceAtWorldPoint(body, force, point);
  return JS_UNDEFINED;
}

static JSValue js_body_apply_force_at_local_point(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpVect force = js2cpvect(js, argv[0]);
  cpVect point = js2cpvect(js, argv[1]);
  cpBodyApplyForceAtLocalPoint(body, force, point);
  return JS_UNDEFINED;
}

static JSValue js_body_apply_impulse_at_world_point(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpVect impulse = js2cpvect(js, argv[0]);
  cpVect point = js2cpvect(js, argv[1]);
  cpBodyApplyImpulseAtWorldPoint(body, impulse, point);
  return JS_UNDEFINED;
}

static JSValue js_body_apply_impulse_at_local_point(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpVect impulse = js2cpvect(js, argv[0]);
  cpVect point = js2cpvect(js, argv[1]);
  cpBodyApplyImpulseAtLocalPoint(body, impulse, point);
  return JS_UNDEFINED;
}

static JSValue js_body_is_sleeping(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  return JS_NewBool(js, cpBodyIsSleeping(body));
}

static JSValue js_body_activate(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpBodyActivate(body);
  return JS_UNDEFINED;
}

static JSValue js_body_sleep(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpBodySleep(body);
  return JS_UNDEFINED;
}

static JSValue js_body_activate_static(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpShape *filter = JS_IsUndefined(argv[0]) ? NULL : JS_GetOpaque(argv[0], js_cpshape_class_id);
  cpBodyActivateStatic(body, filter);
  return JS_UNDEFINED;
}

static JSValue js_body_sleep_with_group(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpBody *group = js2cpBody(js, argv[0]); if (!group) return JS_EXCEPTION;
  cpBodySleepWithGroup(body, group);
  return JS_UNDEFINED;
}

void body_shape_fn(cpBody *body, cpShape *shape, JSValue *fn) {
  JSValue v = *(JSValue*)cpShapeGetUserData(shape);
//  JS_Call(js, *fn, JS_UNDEFINED, 1, &v);
}

static JSValue js_body_each_shape(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  //cpBodyEachShape(body, body_shape_fn, &argv[0]);
  return JS_UNDEFINED;
}

void body_constraint_fn(cpBody *body, cpConstraint *constraint, JSValue *fn) {
  JSValue v = *(JSValue*)cpConstraintGetUserData(constraint);
//  JS_Call(js, *fn, JS_UNDEFINED, 1, &v);
}

static JSValue js_body_each_constraint(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  //cpBodyEachConstraint(body, body_constraint_fn, &argv[0]);
  return JS_UNDEFINED;
}

void body_arbiter_fn(cpBody *body, cpArbiter *arbiter, JSValue *fn) {
  JSValue v = *(JSValue*)cpArbiterGetUserData(arbiter);
//  JS_Call(js, *fn, JS_UNDEFINED, 1, &v);
}

static JSValue js_body_each_arbiter(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
//  cpBodyEachArbiter(body, body_arbiter_fn, &argv[0]);
  return JS_UNDEFINED;
}

static const JSCFunctionListEntry js_body_funcs[] = {
  JS_CGETSET_DEF("position", js_body_get_Position, js_body_set_Position),
  JS_CGETSET_DEF("angle", js_body_get_Angle, js_body_set_Angle),
  JS_CGETSET_DEF("velocity", js_body_get_Velocity, js_body_set_Velocity),
  JS_CGETSET_DEF("angularVelocity", js_body_get_AngularVelocity, js_body_set_AngularVelocity),
  JS_CGETSET_DEF("moment", js_body_get_Moment, js_body_set_Moment),
  JS_CGETSET_DEF("torque", js_body_get_Torque, js_body_set_Torque),
  JS_CGETSET_DEF("mass", js_body_get_Mass, js_body_set_Mass),
  JS_CGETSET_DEF("centerOfGravity", js_body_get_CenterOfGravity, js_body_set_CenterOfGravity),
  JS_CGETSET_DEF("force", js_body_get_Force, js_body_set_Force),
  JS_CGETSET_DEF("type", js_body_get_Type, js_body_set_Type),
  JS_CFUNC_DEF("isSleeping", 0, js_body_is_sleeping),
  JS_CFUNC_DEF("activate", 0, js_body_activate),
  JS_CFUNC_DEF("sleep", 0, js_body_sleep),
  JS_CFUNC_DEF("activateStatic", 1, js_body_activate_static),
  JS_CFUNC_DEF("sleepWithGroup", 1, js_body_sleep_with_group),
  JS_CFUNC_DEF("applyForceAtWorldPoint", 2, js_body_apply_force_at_world_point),
  JS_CFUNC_DEF("applyForceAtLocalPoint", 2, js_body_apply_force_at_local_point),
  JS_CFUNC_DEF("applyImpulseAtWorldPoint", 2, js_body_apply_impulse_at_world_point),
  JS_CFUNC_DEF("applyImpulseAtLocalPoint", 2, js_body_apply_impulse_at_local_point),
  JS_CFUNC_DEF("eachShape", 1, js_body_each_shape),
  JS_CFUNC_DEF("eachConstraint", 1, js_body_each_constraint),
  JS_CFUNC_DEF("eachArbiter", 1, js_body_each_arbiter)
};

// Function definitions

static JSValue circle_proto;
static JSValue segment_proto;
static JSValue poly_proto;

static JSValue js_body_add_circle_shape(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpShape *shape = cpCircleShapeNew(body, js2number(js, argv[0]), js2cpvect(js, argv[1]));
  JSValue obj = JS_NewObjectClass(js, js_cpshape_class_id);
  JS_SetOpaque(obj, shape);
  JS_SetPrototype(js, obj, circle_proto);
  return obj;
}

static JSValue js_body_add_segment_shape(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  cpVect a = js2cpvect(js,argv[0]);
  cpVect b = js2cpvect(js,argv[1]);
  double radius = js2number(js, argv[2]);
  cpShape *shape = cpSegmentShapeNew(body, a, b, radius);
  JSValue obj = JS_NewObjectClass(js, js_cpshape_class_id);
  JS_SetOpaque(obj, shape);
  JS_SetPrototype(js, obj, segment_proto);
  return obj;
}

static JSValue js_body_add_poly_shape(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpBody *body = js2cpBody(js, this_val); if (!body) return JS_EXCEPTION;
  int count = js2number(js, argv[0]);
  cpVect verts[1];
  //cpVect *verts = js2cpvect_array(argv[1], count);
  cpVect offset = js2cpvect(js,argv[2]);
  cpTransform T = {0};
  cpShape *shape = cpPolyShapeNew(body, count, verts, T, 0.0);
  JSValue obj = JS_NewObjectClass(js, js_cpshape_class_id);
  JS_SetOpaque(obj, shape);
  JS_SetPrototype(js, obj, poly_proto);
  return obj;
}

static JSValue js_circle2d_set_radius(JSContext *js, JSValueConst this_val, JSValue val) {
  cpCircleShape *shape = (cpCircleShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  cpCircleShapeSetRadius(shape, js2number(js,val));
  return JS_UNDEFINED;
}

static JSValue js_circle2d_get_radius(JSContext *js, JSValueConst this_val) {
  cpCircleShape *shape = (cpCircleShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  return JS_NewFloat64(js, cpCircleShapeGetRadius(shape));
}

static JSValue js_circle2d_set_offset(JSContext *js, JSValueConst this_val, JSValue val) {
  cpCircleShape *shape = (cpCircleShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  cpCircleShapeSetOffset(shape, js2cpvect(js, val));
  return JS_UNDEFINED;
}

static JSValue js_circle2d_get_offset(JSContext *js, JSValueConst this_val) {
  cpCircleShape *shape = (cpCircleShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  return cpvect2js(js, cpCircleShapeGetOffset(shape));
}

static const JSCFunctionListEntry js_circle2d_funcs[] = {
  JS_CGETSET_DEF("radius", js_circle2d_get_radius, js_circle2d_set_radius),
  JS_CGETSET_DEF("offset", js_circle2d_get_offset, js_circle2d_set_offset)
};

static JSValue js_seg2d_set_endpoints(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpSegmentShape *shape = (cpSegmentShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  cpSegmentShapeSetEndpoints(shape, js2cpvect(js, argv[0]), js2cpvect(js, argv[1]));
  return JS_UNDEFINED;
}

static JSValue js_seg2d_set_radius(JSContext *js, JSValueConst this_val, JSValue val) {
  cpSegmentShape *shape = (cpSegmentShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  cpSegmentShapeSetRadius(shape, js2number(js,val));
  return JS_UNDEFINED;
}

static JSValue js_seg2d_get_radius(JSContext *js, JSValueConst this_val) {
  cpSegmentShape *shape = (cpSegmentShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  return JS_NewFloat64(js, cpSegmentShapeGetRadius(shape));
}

static const JSCFunctionListEntry js_seg2d_funcs[] = {
  JS_CFUNC_DEF("setEndpoints", 2, js_seg2d_set_endpoints),
  JS_CGETSET_DEF("radius", js_seg2d_get_radius, js_seg2d_set_radius)
};

static JSValue js_poly2d_set_verts(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpPolyShape *shape = (cpPolyShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  int count = js2number(js, argv[0]);
//  cpVect *verts = js2cpvect_array(argv[1], count);
  cpVect verts[1];
  cpPolyShapeSetVerts(shape, count, verts, cpTransformIdentity);
  return JS_UNDEFINED;
}

static JSValue js_poly2d_set_radius(JSContext *js, JSValueConst this_val, JSValue val) {
  cpPolyShape *shape = (cpPolyShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  cpPolyShapeSetRadius(shape, js2number(js,val));
  return JS_UNDEFINED;
}

static JSValue js_poly2d_get_radius(JSContext *js, JSValueConst this_val) {
  cpPolyShape *shape = (cpPolyShape *)js2cpShape(js, this_val);
  if (!shape) return JS_EXCEPTION;
  return JS_NewFloat64(js, cpPolyShapeGetRadius(shape));
}

static const JSCFunctionListEntry js_poly2d_funcs[] = {
  JS_CFUNC_DEF("setVerts", 2, js_poly2d_set_verts),
  JS_CGETSET_DEF("radius", js_poly2d_get_radius, js_poly2d_set_radius)
};

////// CPSHAPE //////

static JSValue js_shape_set_filter(JSContext *js, JSValueConst this_val, JSValue val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  cpShapeFilter filter;
  // Assuming the filter is provided as an object with appropriate properties.
  JSValue categories = JS_GetPropertyStr(js, val, "categories");
  JSValue mask = JS_GetPropertyStr(js, val, "mask");
  JSValue group = JS_GetPropertyStr(js, val, "group");
  filter.categories = (uint32_t)js2number(js, categories);
  filter.mask = (uint32_t)js2number(js, mask);
  filter.group = (uint32_t)js2number(js, group);
  JS_FreeValue(js, categories);
  JS_FreeValue(js, mask);
  JS_FreeValue(js, group);
  cpShapeSetFilter(shape, filter);
  return JS_UNDEFINED;
}

static JSValue js_shape_set_sensor(JSContext *js, JSValueConst this_val, JSValue val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  cpShapeSetSensor(shape, JS_ToBool(js, val));
  return JS_UNDEFINED;
}

static JSValue js_shape_get_sensor(JSContext *js, JSValueConst this_val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  return JS_NewBool(js, cpShapeGetSensor(shape));
}

static JSValue js_shape_set_elasticity(JSContext *js, JSValueConst this_val, JSValue val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  cpShapeSetElasticity(shape, js2number(js, val));
  return JS_UNDEFINED;
}

static JSValue js_shape_get_elasticity(JSContext *js, JSValueConst this_val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  return number2js(js, cpShapeGetElasticity(shape));
}

static JSValue js_shape_set_friction(JSContext *js, JSValueConst this_val, JSValue val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  cpShapeSetFriction(shape, js2number(js, val));
  return JS_UNDEFINED;
}

static JSValue js_shape_get_friction(JSContext *js, JSValueConst this_val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  return number2js(js, cpShapeGetFriction(shape));
}

static JSValue js_shape_set_surface_velocity(JSContext *js, JSValueConst this_val, JSValue val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  cpVect velocity = js2cpvect(js, val);
  cpShapeSetSurfaceVelocity(shape, velocity);
  return JS_UNDEFINED;
}

static JSValue js_shape_get_surface_velocity(JSContext *js, JSValueConst this_val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  return cpvect2js(js, cpShapeGetSurfaceVelocity(shape));
}

static JSValue js_shape_get_collision_type(JSContext *js, JSValueConst this_val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  return JS_NewInt32(js, cpShapeGetCollisionType(shape));
}

static JSValue js_shape_set_collision_type(JSContext *js, JSValueConst this_val, JSValue val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  int32_t collision_type;
  if (JS_ToInt32(js, &collision_type, val)) {
    return JS_EXCEPTION;
  }
  cpShapeSetCollisionType(shape, collision_type);
  return JS_UNDEFINED;
}

static JSValue js_shape_get_bb(JSContext *js, JSValueConst this_val, int argc, JSValueConst *argv) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  cpBB bb = cpShapeGetBB(shape);
  JSValue obj = JS_NewObject(js);
  JS_SetPropertyStr(js, obj, "x", JS_NewFloat64(js, bb.l));
  JS_SetPropertyStr(js, obj, "y", JS_NewFloat64(js, bb.b));
  JS_SetPropertyStr(js, obj, "width", JS_NewFloat64(js, bb.r - bb.l));
  JS_SetPropertyStr(js, obj, "height", JS_NewFloat64(js, bb.t - bb.b));
  return obj;
}

static JSValue js_shape_get_filter(JSContext *js, JSValueConst this_val) {
  cpShape *shape = js2cpShape(js, this_val); if (!shape) return JS_EXCEPTION;
  cpShapeFilter filter = cpShapeGetFilter(shape);
  JSValue obj = JS_NewObject(js);
  JS_SetPropertyStr(js, obj, "categories", JS_NewUint32(js, filter.categories));
  JS_SetPropertyStr(js, obj, "mask", JS_NewUint32(js, filter.mask));
  JS_SetPropertyStr(js, obj, "group", JS_NewUint32(js, filter.group));
  return obj;
}

static const JSCFunctionListEntry js_cpshape_funcs[] = {
  JS_CGETSET_DEF("collisionType", js_shape_get_collision_type, js_shape_set_collision_type),
  JS_CFUNC_DEF("getBB", 0, js_shape_get_bb),
  JS_CGETSET_DEF("sensor", js_shape_get_sensor, js_shape_set_sensor),
  JS_CGETSET_DEF("elasticity", js_shape_get_elasticity, js_shape_set_elasticity),
  JS_CGETSET_DEF("friction", js_shape_get_friction, js_shape_set_friction),
  JS_CGETSET_DEF("surfaceVelocity", js_shape_get_surface_velocity, js_shape_set_surface_velocity),
  JS_CGETSET_DEF("filter", js_shape_get_filter, js_shape_set_filter)
};

static JSClassDef js_shape_class = {
  "Shape"
};

////////////////// JOINTS ///

#define CC_GETSET(CPTYPE, ENTRY, TYPE) \
static JSValue js_##CPTYPE##_get_##ENTRY GETSIG { \
  return TYPE##2js(js, CPTYPE##Get##ENTRY(js2cpConstraint(js, this_val))); \
} \
static JSValue js_##CPTYPE##_set_##ENTRY SETSIG { \
  CPTYPE##Set##ENTRY(js2cpConstraint(js, this_val), js2##TYPE(js, val)); \
  return JS_UNDEFINED; \
} \

CC_GETSET(cpConstraint, MaxForce, number)
CC_GETSET(cpConstraint, MaxBias, number)
CC_GETSET(cpConstraint, ErrorBias, number)
CC_GETSET(cpConstraint, CollideBodies, number)

#define JS_FN(NAME, ...) \
static JSValue js_##NAME FNSIG { \
  __VA_ARGS__ ; \
  return JS_UNDEFINED; \
} \

JS_FN(cpConstraint_broken,
  return JS_NewBool(js,cpSpaceContainsConstraint(js2cpSpace(js,this_val), js2cpConstraint(js, argv[0])));
)

JS_FN(cpConstraint_break,
  if (cpSpaceContainsConstraint(js2cpSpace(js,this_val), js2cpConstraint(js, argv[0])))
    cpSpaceRemoveConstraint(js2cpSpace(js,this_val), js2cpConstraint(js, argv[0]));
)

JS_FN(cpConstraint_bodyA,
  cpBody *b = cpConstraintGetBodyA(js2cpConstraint(js, this_val)); 
  return JS_DupValue(js, cpBody2js(js, b)); 
)

JS_FN(cpConstraint_bodyB, 
  cpBody *b = cpConstraintGetBodyB(js2cpConstraint(js, this_val)); 
  return JS_DupValue(js, cpBody2js(js, b)); 
)

static const JSCFunctionListEntry js_cpConstraint_funcs[] = {
  JS_CFUNC_DEF("bodyA", 0, js_cpConstraint_bodyA),
  JS_CFUNC_DEF("bodyB", 0, js_cpConstraint_bodyB),
  JS_CGETSET_DEF("max_force", js_cpConstraint_get_MaxForce, js_cpConstraint_set_MaxForce),
  JS_CGETSET_DEF("max_bias", js_cpConstraint_get_MaxBias, js_cpConstraint_set_MaxBias),
  JS_CGETSET_DEF("error_bias", js_cpConstraint_get_ErrorBias, js_cpConstraint_set_ErrorBias),
  JS_CGETSET_DEF("collide_bodies", js_cpConstraint_get_CollideBodies, js_cpConstraint_set_CollideBodies),
  JS_CFUNC_DEF("broken", 0, js_cpConstraint_broken),
  JS_CFUNC_DEF("break", 0, js_cpConstraint_break)
};

JSValue prep_constraint(JSContext *js, cpConstraint *c)
{
  JSValue ret = cpConstraint2js(js, c);
  JSValue *cb = malloc(sizeof(JSValue));
  *cb = ret;
  cpConstraintSetUserData(c, cb);
  return ret;
}
CC_GETSET(cpPinJoint, Dist, number)
CC_GETSET(cpPinJoint, AnchorA, cpvect)
CC_GETSET(cpPinJoint, AnchorB, cpvect)

static const JSCFunctionListEntry js_pin_funcs[] = {
  JS_CGETSET_DEF("distance", js_cpPinJoint_get_Dist, js_cpPinJoint_set_Dist),
  JS_CGETSET_DEF("anchor_a", js_cpPinJoint_get_AnchorA, js_cpPinJoint_set_AnchorA),
  JS_CGETSET_DEF("anchor_b", js_cpPinJoint_get_AnchorB, js_cpPinJoint_set_AnchorB)
};

JS_FN(joint_pin, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_pin); 
  JSValue pin = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpPinJointNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), cpvzero, cpvzero))); 
  JS_SetPrototype(js, pin, js_pin); 
  return pin;
)

CC_GETSET(cpPivotJoint, AnchorA, cpvect)
CC_GETSET(cpPivotJoint, AnchorB, cpvect)

static const JSCFunctionListEntry js_pivot_funcs[] = {
  JS_CGETSET_DEF("anchor_a", js_cpPivotJoint_get_AnchorA, js_cpPivotJoint_set_AnchorA),
  JS_CGETSET_DEF("anchor_b", js_cpPivotJoint_get_AnchorB, js_cpPivotJoint_set_AnchorB)
};

JS_FN(joint_pivot, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_pivot); 
  JSValue pivot = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpPivotJointNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2cpvect(js, argv[2])))); 
  JS_SetPrototype(js, pivot, js_pivot); 
  return pivot;
)

CC_GETSET(cpGearJoint, Phase, number)
CC_GETSET(cpGearJoint, Ratio, number)

static const JSCFunctionListEntry js_gear_funcs[] = {
  JS_CGETSET_DEF("phase", js_cpGearJoint_get_Phase, js_cpGearJoint_set_Phase),
  JS_CGETSET_DEF("ratio", js_cpGearJoint_get_Ratio, js_cpGearJoint_set_Ratio)
};

JS_FN(joint_gear, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_gear); 
  JSValue gear = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpGearJointNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2number(js, argv[2]), js2number(js, argv[3])))); 
  JS_SetPrototype(js, gear, js_gear); 
  return gear;
)

CC_GETSET(cpRotaryLimitJoint, Min, number)
CC_GETSET(cpRotaryLimitJoint, Max, number)

static const JSCFunctionListEntry js_rotary_funcs[] = {
  JS_CGETSET_DEF("min", js_cpRotaryLimitJoint_get_Min, js_cpRotaryLimitJoint_set_Min),
  JS_CGETSET_DEF("max", js_cpRotaryLimitJoint_get_Max, js_cpRotaryLimitJoint_set_Max)
};

JS_FN(joint_rotary, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_rotary); 
  JSValue rotary = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpRotaryLimitJointNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2number(js, argv[2]), js2number(js, argv[3])))); 
  JS_SetPrototype(js, rotary, js_rotary); 
  return rotary;
)

CC_GETSET(cpDampedRotarySpring, RestAngle, number)
CC_GETSET(cpDampedRotarySpring, Stiffness, number)
CC_GETSET(cpDampedRotarySpring, Damping, number)

static const JSCFunctionListEntry js_damped_rotary_funcs[] = {
  JS_CGETSET_DEF("rest_angle", js_cpDampedRotarySpring_get_RestAngle, js_cpDampedRotarySpring_set_RestAngle),
  JS_CGETSET_DEF("stiffness", js_cpDampedRotarySpring_get_Stiffness, js_cpDampedRotarySpring_set_Stiffness),
  JS_CGETSET_DEF("damping", js_cpDampedRotarySpring_get_Damping, js_cpDampedRotarySpring_set_Damping)
};

JS_FN(joint_damped_rotary, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_damped_rotary); 
  JSValue damped_rotary = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpDampedRotarySpringNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2number(js, argv[2]), js2number(js, argv[3]), js2number(js, argv[4])))); 
  JS_SetPrototype(js, damped_rotary, js_damped_rotary); 
  return damped_rotary;
)

CC_GETSET(cpDampedSpring, AnchorA, cpvect)
CC_GETSET(cpDampedSpring, AnchorB, cpvect)
CC_GETSET(cpDampedSpring, RestLength, number)
CC_GETSET(cpDampedSpring, Stiffness, number)
CC_GETSET(cpDampedSpring, Damping, number)

static const JSCFunctionListEntry js_damped_spring_funcs[] = {
  JS_CGETSET_DEF("anchor_a", js_cpDampedSpring_get_AnchorA, js_cpDampedSpring_set_AnchorA),
  JS_CGETSET_DEF("anchor_b", js_cpDampedSpring_get_AnchorB, js_cpDampedSpring_set_AnchorB),
  JS_CGETSET_DEF("rest_length", js_cpDampedSpring_get_RestLength, js_cpDampedSpring_set_RestLength),
  JS_CGETSET_DEF("stiffness", js_cpDampedSpring_get_Stiffness, js_cpDampedSpring_set_Stiffness),
  JS_CGETSET_DEF("damping", js_cpDampedSpring_get_Damping, js_cpDampedSpring_set_Damping)
};

JS_FN(joint_damped_spring, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_damped_spring); 
  JSValue damped_spring = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpDampedSpringNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2cpvect(js, argv[2]), js2cpvect(js, argv[3]), js2number(js, argv[4]), js2number(js, argv[5]), js2number(js, argv[6])))); 
  JS_SetPrototype(js, damped_spring, js_damped_spring); 
  return damped_spring;
)

CC_GETSET(cpGrooveJoint, GrooveA, cpvect)
CC_GETSET(cpGrooveJoint, GrooveB, cpvect)
CC_GETSET(cpGrooveJoint, AnchorB, cpvect)

static const JSCFunctionListEntry js_groove_funcs[] = {
  JS_CGETSET_DEF("groove_a", js_cpGrooveJoint_get_GrooveA, js_cpGrooveJoint_set_GrooveA),
  JS_CGETSET_DEF("groove_b", js_cpGrooveJoint_get_GrooveB, js_cpGrooveJoint_set_GrooveB),
  JS_CGETSET_DEF("anchor_b", js_cpGrooveJoint_get_AnchorB, js_cpGrooveJoint_set_AnchorB)
};

JS_FN(joint_groove, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_groove); 
  JSValue groove = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpGrooveJointNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2cpvect(js, argv[2]), js2cpvect(js, argv[3]), js2cpvect(js, argv[4])))); 
  JS_SetPrototype(js, groove, js_groove); 
  return groove;
)

CC_GETSET(cpSlideJoint, AnchorA, cpvect)
CC_GETSET(cpSlideJoint, AnchorB, cpvect)
CC_GETSET(cpSlideJoint, Min, number)
CC_GETSET(cpSlideJoint, Max, number)

static const JSCFunctionListEntry js_slide_funcs[] = {
  JS_CGETSET_DEF("anchor_a", js_cpSlideJoint_get_AnchorA, js_cpSlideJoint_set_AnchorA),
  JS_CGETSET_DEF("anchor_b", js_cpSlideJoint_get_AnchorB, js_cpSlideJoint_set_AnchorB),
  JS_CGETSET_DEF("min", js_cpSlideJoint_get_Min, js_cpSlideJoint_set_Min),
  JS_CGETSET_DEF("max", js_cpSlideJoint_get_Max, js_cpSlideJoint_set_Max)
};

JS_FN(joint_slide, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_slide); 
  JSValue slide = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpSlideJointNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2cpvect(js, argv[2]), js2cpvect(js, argv[3]), js2number(js, argv[4]), js2number(js, argv[5])))); 
  JS_SetPrototype(js, slide, js_slide); 
  return slide;
)

CC_GETSET(cpRatchetJoint, Angle, number)
CC_GETSET(cpRatchetJoint, Phase, number)
CC_GETSET(cpRatchetJoint, Ratchet, number)

static const JSCFunctionListEntry js_ratchet_funcs[] = {
  JS_CGETSET_DEF("angle", js_cpRatchetJoint_get_Angle, js_cpRatchetJoint_set_Angle),
  JS_CGETSET_DEF("phase", js_cpRatchetJoint_get_Phase, js_cpRatchetJoint_set_Phase),
  JS_CGETSET_DEF("ratchet", js_cpRatchetJoint_get_Ratchet, js_cpRatchetJoint_set_Ratchet)
};

JS_FN(joint_ratchet, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_ratchet); 
  JSValue ratchet = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpRatchetJointNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2number(js, argv[2]), js2number(js, argv[3])))); 
  JS_SetPrototype(js, ratchet, js_ratchet); 
  return ratchet;
)

CC_GETSET(cpSimpleMotor, Rate, number)

static const JSCFunctionListEntry js_motor_funcs[] = {
  JS_CGETSET_DEF("rate", js_cpSimpleMotor_get_Rate, js_cpSimpleMotor_set_Rate)
};

JS_FN(joint_motor, 
  if (JS_IsUndefined(argv[0])) return JS_DupValue(js, js_motor); 
  JSValue motor = prep_constraint(js, cpSpaceAddConstraint(js2cpSpace(js, this_val), cpSimpleMotorNew(js2cpBody(js, argv[0]), js2cpBody(js, argv[1]), js2number(js, argv[2])))); 
  JS_SetPrototype(js, motor, js_motor);
  return motor;
)

static const JSCFunctionListEntry js_joint_funcs[] = {
  JS_CFUNC_DEF("pin", 2, js_joint_pin),
  JS_CFUNC_DEF("pivot", 3, js_joint_pivot),
  JS_CFUNC_DEF("gear", 4, js_joint_gear),
  JS_CFUNC_DEF("rotary", 4, js_joint_rotary),
  JS_CFUNC_DEF("damped_rotary", 5, js_joint_damped_rotary),
  JS_CFUNC_DEF("damped_spring", 7, js_joint_damped_spring),
  JS_CFUNC_DEF("groove", 5, js_joint_groove),
  JS_CFUNC_DEF("slide", 6, js_joint_slide),
  JS_CFUNC_DEF("ratchet", 4, js_joint_ratchet),
  JS_CFUNC_DEF("motor", 3, js_joint_motor)
};

#define JSSTATIC(NAME, PARENT) \
js_##NAME = JS_NewObject(js); \
JS_SetPropertyFunctionList(js, js_##NAME, js_##NAME##_funcs, countof(js_##NAME##_funcs)); \
JS_SetPrototype(js, js_##NAME, PARENT); \

static int js_init_module(JSContext *js, JSModuleDef *m) {
  JS_NewClassID(&js_cpspace_class_id);
  JS_NewClassID(&js_cpbody_class_id);
  JS_NewClassID(&js_cpshape_class_id);

  JS_NewClass(JS_GetRuntime(js), js_cpspace_class_id, &js_space_class);
  JS_NewClass(JS_GetRuntime(js), js_cpbody_class_id, &js_space_class);
  JS_NewClass(JS_GetRuntime(js), js_cpshape_class_id, &js_shape_class);

  JSValue shape_proto = JS_NewObject(js);
  JS_SetPropertyFunctionList(js, shape_proto, js_cpshape_funcs, countof(js_cpshape_funcs));
  JS_SetClassProto(js, js_cpshape_class_id, shape_proto);
  
  circle_proto = JS_NewObject(js);
  JS_SetPropertyFunctionList(js, circle_proto, js_circle2d_funcs, countof(js_circle2d_funcs));
  JS_SetPrototype(js, circle_proto, shape_proto);
  
  segment_proto = JS_NewObject(js);
  JS_SetPropertyFunctionList(js, segment_proto, js_seg2d_funcs, countof(js_seg2d_funcs));
  JS_SetPrototype(js, segment_proto, shape_proto);
  
  poly_proto = JS_NewObject(js);
  JS_SetPropertyFunctionList(js, poly_proto, js_poly2d_funcs, countof(js_poly2d_funcs));
  JS_SetPrototype(js, poly_proto, shape_proto);

  JSValue space_proto = JS_NewObject(js);
  JS_SetPropertyFunctionList(js, space_proto, js_body_funcs, countof(js_body_funcs));
  
  /*JSSTATIC(pin, cpConstraint_proto)
  JSSTATIC(motor, cpConstraint_proto)
  JSSTATIC(ratchet, cpConstraint_proto)
  JSSTATIC(slide, cpConstraint_proto)
  JSSTATIC(pivot, cpConstraint_proto)
  JSSTATIC(gear, cpConstraint_proto)
  JSSTATIC(rotary, cpConstraint_proto)
  JSSTATIC(damped_rotary, cpConstraint_proto)
  JSSTATIC(damped_spring, cpConstraint_proto)
  JSSTATIC(groove, cpConstraint_proto)*/

  JSValue cpspace_class = JS_NewCFunction2(js, js_make_space, "Space", 0, JS_CFUNC_constructor, 0);
  JS_SetClassProto(js, js_cpspace_class_id, space_proto);
  JS_SetModuleExport(js, m, "Space", cpspace_class);

  return 0;
}

#ifdef JS_SHARED_LIBRARY
#define JS_INIT_MODULE js_init_module
#else
#define JS_INIT_MODULE js_init_module_chipmunk
#endif

JSModuleDef *JS_INIT_MODULE(JSContext *js, const char *name) {
  JSModuleDef *m;
  m = JS_NewCModule(js, name, js_init_module);
  if (!m)
    return NULL;
  JS_AddModuleExport(js, m, "Space");
  return m;
}
