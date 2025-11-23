#include <cglm/vec2.h>
#include <raylib.h>
#include <elc/core.h>
#include <stdlib.h>

typedef struct Particle {
    vec2 position;
    vec2 velocity;
} Particle;

typedef struct DistanceConstraint {
    Particle* particle_a;
    Particle* particle_b;
    float distance;
} DistanceConstraint;

typedef struct OriginConstraint {
    Particle* particle;
    float distance;
} OriginConstraint;

typedef struct ParticleTrail {
    Vector2* points;
    u32 n_points, max_points;
} ParticleTrail;

float originConstraint(OriginConstraint constraint) {
    return fabsf(glm_vec2_norm(constraint.particle->position)) - constraint.distance;
}

void originJacobian(vec2 point, vec2 jacobian /* using `vec2` as a `mat1x2` */) {
    float norm = fabsf(glm_vec2_norm(point)); /* length of `point` */
    jacobian[0] = point[0] / norm; /* x / sqrt(x^2 + y^2) */
    jacobian[1] = point[1] / norm; /* y / sqrt(x^2 + y^2) */
}

float originViolation(OriginConstraint constraint, vec2 jacobian, float dt) {
    return -glm_vec2_dot(jacobian, constraint.particle->velocity) - originConstraint(constraint) / dt;
}

void applyOriginConstraint(OriginConstraint constraint, float dt) {
    vec2 jacobian;
    originJacobian(constraint.particle->position, jacobian);
    glm_vec2_muladds(jacobian, originViolation(constraint, jacobian, dt) / glm_vec2_dot(jacobian, jacobian), constraint.particle->velocity);
}

float distanceConstraint(DistanceConstraint constraint) {
    return fabsf(glm_vec2_distance(constraint.particle_a->position, constraint.particle_b->position)) - constraint.distance;
}

void distanceJacobian(vec2 point_a, vec2 point_b, vec4 jacobian) {
    float norm = fabsf(glm_vec2_distance(point_a, point_b));
    jacobian[0] = -((point_b[0] - point_a[0]) / norm);
    jacobian[1] = -((point_b[1] - point_a[1]) / norm);
    jacobian[2] = (point_b[0] - point_a[0]) / norm;
    jacobian[3] = (point_b[1] - point_a[1]) / norm;
}

float distanceViolation(DistanceConstraint constraint, vec4 jacobian, float dt) {
    return -glm_vec4_dot(jacobian, (vec4){constraint.particle_a->velocity[0], constraint.particle_a->velocity[1], constraint.particle_b->velocity[0], constraint.particle_b->velocity[1]}) - distanceConstraint(constraint) / dt;
}

void applyDistanceConstraint(DistanceConstraint constraint, float dt) {
    vec4 jacobian;
    distanceJacobian(constraint.particle_a->position, constraint.particle_b->position, jacobian);
    glm_vec2_muladds(&jacobian[0], distanceViolation(constraint, jacobian, dt) / glm_vec4_dot(jacobian, jacobian), constraint.particle_a->velocity);
    glm_vec2_muladds(&jacobian[2], distanceViolation(constraint, jacobian, dt) / glm_vec4_dot(jacobian, jacobian), constraint.particle_b->velocity);
}

void applyParticleGravity(Particle* particle, float dt /* delta time */) {
    particle->velocity[1] += 9.8f * dt; /* apply gravity * delta time */
}

void applyParticleVelocity(Particle* particle, float dt /* delta time */) {
    glm_vec2_muladds(particle->velocity, dt, particle->position);
}

ParticleTrail createParticleTrail(u32 max_points) {
    return (ParticleTrail){.max_points = max_points, .points = malloc(max_points * sizeof(Vector2))};
}

void resizeParticleTrail(ParticleTrail* trail, u32 max_points) {
    trail->max_points = max_points;
    trail->points = realloc(trail->points, max_points * sizeof(Vector2));
}

void particleTrailAddPoint(ParticleTrail* trail, vec2 point) {
    if (trail->n_points + 1 > trail->max_points) resizeParticleTrail(trail, (trail->max_points * 1.5f) + 1);
    trail->points[trail->n_points++] = (Vector2){((float)800 / 2) + (point[0] * 50), ((float)600 / 2) + (point[1] * 50)};
}

void drawParticleTrail(ParticleTrail trail) {
    DrawLineStrip(trail.points, trail.n_points, WHITE);
}

int main() {
    InitWindow(800, 600, "constraint solver");
    SetTargetFPS(240);

    Particle particle_a = {.position = {1.0f}};
    Particle particle_b = {.position = {2.0f}};
    Particle particle_c = {.position = {3.0f}};
    OriginConstraint origin_constraint = {.distance = 1.0f, .particle = &particle_a};
    DistanceConstraint distance_constraint_a = {.distance = 1.0f, .particle_a = &particle_a, .particle_b = &particle_b};
    DistanceConstraint distance_constraint_b = {.distance = 1.0f, .particle_a = &particle_b, .particle_b = &particle_c};

    ParticleTrail trail = createParticleTrail(100);

    while (!WindowShouldClose()) {
        float dt = 1.0f / 240.0f;

        particleTrailAddPoint(&trail, particle_c.position);

        applyParticleGravity(&particle_a, dt);
        applyParticleGravity(&particle_b, dt);
        applyParticleGravity(&particle_c, dt);

        for (u32 j = 0; j < 20; j++) {
            applyOriginConstraint(origin_constraint, dt);
            applyDistanceConstraint(distance_constraint_a, dt);
            applyDistanceConstraint(distance_constraint_b, dt);
        }

        applyParticleVelocity(&particle_a, dt);
        applyParticleVelocity(&particle_b, dt);
        applyParticleVelocity(&particle_c, dt);

        BeginDrawing();

        ClearBackground(BLACK);

        drawParticleTrail(trail);

        DrawCircle(800 / 2, 600 / 2, 5, YELLOW);
        DrawCircle(((float)800 / 2) + (particle_a.position[0] * 50), ((float)600 / 2) + (particle_a.position[1] * 50), 5, BLUE);
        DrawCircle(((float)800 / 2) + (particle_b.position[0] * 50), ((float)600 / 2) + (particle_b.position[1] * 50), 5, RED);
        DrawCircle(((float)800 / 2) + (particle_c.position[0] * 50), ((float)600 / 2) + (particle_c.position[1] * 50), 5, ORANGE);

        DrawFPS(10, 10);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
