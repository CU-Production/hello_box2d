#define SOKOL_IMPL
#define SOKOL_GLCORE33
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_gp.h"
#include "sokol_glue.h"
#include "sokol_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "box2d/box2d.h"

#include "TaskScheduler_c.h"

#define ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )
#define MAYBE_UNUSED( x ) ( (void)( x ) )

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#define MAX_TASKS 128
#define THREAD_LIMIT 32
#define USING_JOB_THREADS 8

struct TaskData
{
    b2TaskCallback* box2dTask;
    void* box2dContext;
};

static struct {
    struct {
        b2Vec2 gravity = {0.0f, -5.0f};
        b2WorldId worldId = b2_nullWorldId;
        float timeStep = 1.0f / 60.0f;
        int32_t subStepCount = 4;
    } b2d_global_ctx;
    struct {
        b2BodyId bodyId = b2_nullBodyId;
        b2Polygon shape;
        const float halfWidth = 50.0f;
        const float halfHeight = 5.0f;
    } s_ground;
    const static int32_t dynamicBoxCount = 100;
    struct {
        b2BodyId bodyId = b2_nullBodyId;
        b2Polygon shape;
        const float halfWidth = 1.0f;
        const float halfHeight = 1.0f;
    } d_box[dynamicBoxCount];

    struct {
        enkiTaskScheduler* scheduler;
        enkiTaskSet* tasks[MAX_TASKS];
        TaskData taskData[MAX_TASKS];
        int taskCount = 0;
    } mt;
} state;

void ExecuteRangeTask( uint32_t start, uint32_t end, uint32_t threadIndex, void* context )
{
    TaskData* data = (TaskData*)context;
    data->box2dTask( start, end, threadIndex, data->box2dContext );
}

static void* EnqueueTask( b2TaskCallback* box2dTask, int itemCount, int minRange, void* box2dContext, void* userContext )
{
    MAYBE_UNUSED( userContext );

    if ( state.mt.taskCount < MAX_TASKS )
    {
        enkiTaskSet* task = state.mt.tasks[state.mt.taskCount];
        TaskData* data = state.mt.taskData + state.mt.taskCount;
        data->box2dTask = box2dTask;
        data->box2dContext = box2dContext;

        enkiParamsTaskSet params{};
        params.minRange = minRange;
        params.setSize = itemCount;
        params.pArgs = data;
        params.priority = 0;

        enkiSetParamsTaskSet( task, params );
        enkiAddTaskSet( state.mt.scheduler, task );

        ++state.mt.taskCount;

        return task;
    }
    else
    {
        printf( "MAX_TASKS exceeded!!!\n" );
        box2dTask( 0, itemCount, 0, box2dContext );
        return NULL;
    }
}

static void FinishTask( void* userTask, void* userContext )
{
    MAYBE_UNUSED( userContext );

    enkiTaskSet* task = (enkiTaskSet*)userTask;
    enkiWaitForTaskSet( state.mt.scheduler, task );
}

static void init(void) {
    sg_desc sgdesc = {};
    sgdesc.context = sapp_sgcontext();
    sgdesc.logger.func = slog_func;

    sg_setup(&sgdesc);
    if(!sg_isvalid()) {
        fprintf(stderr, "Failed to create Sokol GFX context!\n");
        exit(-1);
    }

    sgp_desc sgpdesc = {0};
    sgp_setup(&sgpdesc);
    if(!sgp_is_valid()) {
        fprintf(stderr, "Failed to create Sokol GP context: %s\n", sgp_get_error_message(sgp_get_last_error()));
        exit(-1);
    }

    // enki init
    state.mt.scheduler = enkiNewTaskScheduler();
    enkiTaskSchedulerConfig config = enkiGetTaskSchedulerConfig(state.mt.scheduler);
    config.numTaskThreadsToCreate = USING_JOB_THREADS - 1;
    enkiInitTaskSchedulerWithConfig(state.mt.scheduler, config);

    for ( int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex )
    {
        state.mt.tasks[taskIndex] = enkiCreateTaskSet(state.mt.scheduler, ExecuteRangeTask);
    }

    // world
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = state.b2d_global_ctx.gravity;
    worldDef.enableSleep = false;
    worldDef.enableContinous = true;
    worldDef.enqueueTask = EnqueueTask;
    worldDef.finishTask = FinishTask;
    worldDef.workerCount = USING_JOB_THREADS;

    state.b2d_global_ctx.worldId = b2CreateWorld(&worldDef);

    // ground
    b2BodyDef groundBodyDef = b2DefaultBodyDef();
    groundBodyDef.position = {0.0f, -10.f};

    state.s_ground.bodyId = b2CreateBody(state.b2d_global_ctx.worldId, &groundBodyDef);

    state.s_ground.shape = b2MakeBox(state.s_ground.halfWidth, state.s_ground.halfHeight);

    b2ShapeDef groundShapeDef = b2DefaultShapeDef();
    b2CreatePolygonShape(state.s_ground.bodyId, &groundShapeDef, &state.s_ground.shape);

    // dynamic body
    for (int32_t i = 0; i < state.dynamicBoxCount; i++) {
        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = {0.0f + (fmodf((float)i, 2.0f)), 4.0f + 6.0f * i};

        state.d_box[i].bodyId = b2CreateBody(state.b2d_global_ctx.worldId, &bodyDef);

        state.d_box[i].shape = b2MakeBox(state.d_box[i].halfWidth, state.d_box[i].halfHeight);

        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = 1.0f;
        shapeDef.friction = 0.3f;
        shapeDef.restitution = 0.5f;

        b2CreatePolygonShape(state.d_box[i].bodyId, &shapeDef, &state.d_box[i].shape);
    }
}

static void cleanup(void) {
    b2DestroyWorld(state.b2d_global_ctx.worldId);

    for ( int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex )
    {
        enkiDeleteTaskSet(state.mt.scheduler, state.mt.tasks[taskIndex]);
        state.mt.tasks[taskIndex] = nullptr;
        state.mt.taskData[taskIndex] = {};
    }

    enkiDeleteTaskScheduler(state.mt.scheduler);

    sgp_shutdown();
    sg_shutdown();
}

static void frame(void) {
    int width = sapp_width(), height = sapp_height();
    float ratio = width/(float)height;

    sgp_begin(width, height);
    sgp_viewport(0, 0, width, height);

    const float halfDemoHeight = 30.0f;
    sgp_project(-ratio*halfDemoHeight, ratio*halfDemoHeight, halfDemoHeight, -halfDemoHeight);

    sgp_set_color(0.1f, 0.1f, 0.1f, 1.0f);
    sgp_clear();

    state.b2d_global_ctx.timeStep = static_cast<float>(sapp_frame_duration());
    b2World_Step(state.b2d_global_ctx.worldId, state.b2d_global_ctx.timeStep, state.b2d_global_ctx.subStepCount);
    state.mt.taskCount = 0;

    // draw ground
    {
        b2Vec2 position = b2Body_GetPosition(state.s_ground.bodyId);
        sgp_push_transform();
        sgp_set_color(1.0f, 1.0f, 1.0f, 1.0f);
        sgp_translate(position.x, position.y);
        sgp_draw_filled_rect(-state.s_ground.halfWidth, -state.s_ground.halfHeight, state.s_ground.halfWidth*2, state.s_ground.halfHeight*2);
        sgp_pop_transform();
    }

    // draw dynamic box
    for (int32_t i = 0; i < state.dynamicBoxCount; i++)
    {
        b2Vec2 position = b2Body_GetPosition(state.d_box[i].bodyId);
        float angle = b2Rot_GetAngle(b2Body_GetRotation(state.d_box[i].bodyId));
        sgp_push_transform();
        sgp_translate(position.x, position.y);
        sgp_rotate(angle);
        sgp_set_color(1.0f, 0.0f, 0.0f, 1.0f);
        // sgp_draw_filled_rect(-state.d_box[i].halfWidth, -state.d_box[i].halfHeight, state.d_box[i].halfWidth*2, state.d_box[i].halfHeight*2);
        sgp_draw_line(-state.d_box[i].halfWidth, -state.d_box[i].halfHeight, state.d_box[i].halfWidth, -state.d_box[i].halfHeight);
        sgp_draw_line(-state.d_box[i].halfWidth, -state.d_box[i].halfHeight, -state.d_box[i].halfWidth, state.d_box[i].halfHeight);
        sgp_draw_line(state.d_box[i].halfWidth, -state.d_box[i].halfHeight, state.d_box[i].halfWidth, state.d_box[i].halfHeight);
        sgp_draw_line(-state.d_box[i].halfWidth, state.d_box[i].halfHeight, state.d_box[i].halfWidth, state.d_box[i].halfHeight);
        sgp_draw_line(0, 0, state.d_box[i].halfWidth, 0);
        sgp_pop_transform();
    }

    sg_pass_action pass_action = {0};
    sg_begin_default_pass(&pass_action, width, height);

    sgp_flush();
    sgp_end();

    sg_end_pass();
    sg_commit();
}

sapp_desc sokol_main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    sapp_desc app_desc{};
    app_desc.init_cb = init;
    app_desc.frame_cb = frame;
    app_desc.cleanup_cb = cleanup;
    app_desc.window_title = "hello box2d";
    app_desc.logger.func = slog_func;
    app_desc.width = 1280;
    app_desc.height = 720;
    app_desc.high_dpi = true;
    return app_desc;
}