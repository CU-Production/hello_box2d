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

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

static struct {
    struct {
        b2Vec2 gravity = {0.0f, -5.0f};
        b2World world{gravity};
        float timeStep = 1.0f / 60.0f;
        int32 velocityIterations = 6;
        int32 positionIterations = 2;
    } b2d_global_ctx;
    struct {
        b2Body* body = nullptr;
        b2PolygonShape shape;
        const float halfWidth = 50.0f;
        const float halfHeight = 5.0f;
    } s_ground;
    const static int32 dynamicBoxCount = 100;
    struct {
        b2Body* body = nullptr;
        b2PolygonShape shape;
        const float halfWidth = 1.0f;
        const float halfHeight = 1.0f;
    } d_box[dynamicBoxCount];
} state;

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

    // ground
    b2BodyDef groundBodyDef;
    groundBodyDef.position = {0.0f, -10.f};

    state.s_ground.body = state.b2d_global_ctx.world.CreateBody(&groundBodyDef);

    b2PolygonShape groundBox;
    groundBox.SetAsBox(state.s_ground.halfWidth, state.s_ground.halfHeight);

    state.s_ground.body->CreateFixture(&groundBox, 0.0f);

    // dynamic body
    for (int32 i = 0; i < state.dynamicBoxCount; i++) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = {0.0f + (fmodf((float)i, 2.0f)), 4.0f + 6.0f * i};

        state.d_box[i].body = state.b2d_global_ctx.world.CreateBody(&bodyDef);

        b2PolygonShape dynamicBox;
        dynamicBox.SetAsBox(state.d_box[i].halfWidth, state.d_box[i].halfHeight);

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &dynamicBox;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = 0.3f;
        fixtureDef.restitution = 0.5f;

        state.d_box[i].body->CreateFixture(&fixtureDef);
    }
}

static void cleanup(void) {
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

    state.b2d_global_ctx.timeStep = sapp_frame_duration();
    state.b2d_global_ctx.world.Step(state.b2d_global_ctx.timeStep, state.b2d_global_ctx.velocityIterations, state.b2d_global_ctx.positionIterations);

    // draw ground
    {
        b2Vec2 position = state.s_ground.body->GetPosition();
        sgp_push_transform();
        sgp_set_color(1.0f, 1.0f, 1.0f, 1.0f);
        sgp_translate(position.x, position.y);
        sgp_draw_filled_rect(-state.s_ground.halfWidth, -state.s_ground.halfHeight, state.s_ground.halfWidth*2, state.s_ground.halfHeight*2);
        sgp_pop_transform();
    }

    // draw dynamic box
    for (int32 i = 0; i < state.dynamicBoxCount; i++)
    {
        b2Vec2 position = state.d_box[i].body->GetPosition();
        float angle = state.d_box[i].body->GetAngle();
        sgp_push_transform();
        sgp_translate(position.x, position.y);
        sgp_rotate(angle);
        sgp_set_color(1.0f, 0.0f, 0.0f, 1.0f);
        sgp_draw_filled_rect(-state.d_box[i].halfWidth, -state.d_box[i].halfHeight, state.d_box[i].halfWidth*2, state.d_box[i].halfHeight*2);
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