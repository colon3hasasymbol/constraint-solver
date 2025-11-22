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

void distanceJacobian(vec2 point_a, vec2 point_b, vec2 jacobian) {
    float norm = fabsf(glm_vec2_distance(point_a, point_b));
    jacobian[0] = (point_a[0] - point_b[0]) / norm;
    jacobian[1] = (point_a[1] - point_b[1]) / norm;
}

float distanceViolation(DistanceConstraint constraint, vec2 jacobian, float dt) {
    vec2 relative_velocity;
    glm_vec2_sub(constraint.particle_a->velocity, constraint.particle_b->velocity, relative_velocity);
    return -glm_vec2_dot(jacobian, relative_velocity) - distanceConstraint(constraint) / dt;
}

void applyDistanceConstraint(DistanceConstraint constraint, float dt) {
    vec2 jacobian;
    distanceJacobian(constraint.particle_a->position, constraint.particle_b->position, jacobian);
    glm_vec2_muladds(jacobian, distanceViolation(constraint, jacobian, dt) / glm_vec2_dot(jacobian, jacobian), constraint.particle_a->velocity);
    glm_vec2_mulsubs(jacobian, distanceViolation(constraint, jacobian, dt) / glm_vec2_dot(jacobian, jacobian), constraint.particle_b->velocity);
}

void applyParticleGravity(Particle* particle, float dt /* delta time */) {
    particle->velocity[1] += 9.8f * dt; /* apply gravity * delta time */
}

void applyParticleVelocity(Particle* particle, float dt /* delta time */) {
    glm_vec2_muladds(particle->velocity, dt, particle->position);
}

int main() {
    InitWindow(800, 600, "constraint solver");
    SetTargetFPS(60);

    Particle particle_a = {.position = {1.0f}};
    Particle particle_b = {.position = {2.0f}};
    OriginConstraint origin_constraint = {.distance = 1.0f, .particle = &particle_a};
    DistanceConstraint distance_constraint = {.distance = 1.0f, .particle_a = &particle_a, .particle_b = &particle_b};

    while (!WindowShouldClose()) {
        float dt = 1.0f / 60.0f;

        applyParticleGravity(&particle_a, dt);
        applyParticleGravity(&particle_b, dt);

        for (u8 i = 0; i < 1; i++) {
            applyDistanceConstraint(distance_constraint, dt);
            applyOriginConstraint(origin_constraint, dt);
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
