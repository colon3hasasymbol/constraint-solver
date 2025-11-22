#include <cglm/vec2.h>
#include <raylib.h>
#include <elc/core.h>

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

int main() {
    InitWindow(800, 600, "constraint solver");
    SetTargetFPS(120);

    Particle particle_b = {.position = {2.0f}};
    Particle particle_a = {.position = {1.0f}};
    OriginConstraint origin_constraint = {.distance = 1.0f, .particle = &particle_a};
    DistanceConstraint distance_constraint = {.distance = 1.0f, .particle_a = &particle_a, .particle_b = &particle_b};

    while (!WindowShouldClose()) {
        float dt = 1.0f / 120.0f;

        applyParticleGravity(&particle_a, dt);
        applyParticleGravity(&particle_b, dt);

        for (u32 j = 0; j < 10; j++) {
            applyOriginConstraint(origin_constraint, dt);
            applyDistanceConstraint(distance_constraint, dt);
        }

        applyParticleVelocity(&particle_a, dt);
        applyParticleVelocity(&particle_b, dt);

        BeginDrawing();

        ClearBackground(BLACK);

        DrawCircle(800 / 2, 600 / 2, 5, YELLOW);
        DrawCircle(((float)800 / 2) + (particle_a.position[0] * 50), ((float)600 / 2) + (particle_a.position[1] * 50), 5, BLUE);
        DrawCircle(((float)800 / 2) + (particle_b.position[0] * 50), ((float)600 / 2) + (particle_b.position[1] * 50), 5, RED);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
