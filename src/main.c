#include <cglm/io.h>
#include <cglm/mat4.h>
#include <cglm/util.h>
#include <cglm/vec2.h>
#include <raylib.h>
#include <elc/core.h>
#include <stdlib.h>

typedef struct Particle {
    vec2 position;
    vec2 velocity;
    float mass;
    float rotation;
    float omega;
    float inertia;
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

typedef struct AngleConstraint {
    Particle* particle_a;
    Particle* particle_b;
} AngleConstraint;

typedef struct ParticleTrail {
    Vector2* points;
    u32 n_points, max_points;
} ParticleTrail;

typedef struct PositionSpring {
    Particle* particle;
    vec2 position;
    float distance, stiffness;
} PositionSpring;

void createMassInertiaMatrix(float m_a, float i_a, float m_b, float i_b, mat6 dest) {
    elc_mat6_zero(dest);
    dest[0][0] = m_a;
    dest[1][1] = m_a;
    dest[2][2] = i_a;
    dest[3][3] = m_b;
    dest[4][4] = m_b;
    dest[5][5] = i_b;
}

void createMassMatrix(float m_a, float m_b, mat4 dest) {
    glm_mat4_zero(dest);
    dest[0][0] = m_a;
    dest[1][1] = m_a;
    dest[2][2] = m_b;
    dest[3][3] = m_b;
}

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

float massImpulseDenominator(vec4 jacobian, mat4 mass) {
    vec4 jtm;
    glm_mat4_mulv(mass, jacobian, jtm);
    return glm_vec4_dot(jtm, jacobian);
}

float inertiaImpulseDenominator(vec6 jacobian, mat6 mass) {
    vec6 jtm;
    elc_mat6_mulv(mass, jacobian, jtm);
    return elc_vec6_dot(jtm, jacobian);
}

float distanceViolation(DistanceConstraint constraint, vec4 jacobian, float dt) {
    return -glm_vec4_dot(jacobian, V2V2_TO_V4(constraint.particle_a->velocity, constraint.particle_b->velocity)) - distanceConstraint(constraint) / dt;
}

void applyDistanceConstraint(DistanceConstraint constraint, float dt) {
    vec4 jacobian;
    mat4 mass_matrix;
    createMassMatrix(constraint.particle_a->mass, constraint.particle_b->mass, mass_matrix);
    distanceJacobian(constraint.particle_a->position, constraint.particle_b->position, jacobian);
    float den = massImpulseDenominator(jacobian, mass_matrix);
    glm_vec2_muladds(&jacobian[0], distanceViolation(constraint, jacobian, dt) / den, constraint.particle_a->velocity);
    glm_vec2_muladds(&jacobian[2], distanceViolation(constraint, jacobian, dt) / den, constraint.particle_b->velocity);
}

float angleConstraint(AngleConstraint constraint) {
    return constraint.particle_a->rotation - constraint.particle_b->rotation;
}

void angleJacobian(vec6 jacobian) {
    jacobian[0] = 0.0f;
    jacobian[1] = 0.0f;
    jacobian[2] = 1.0f;
    jacobian[3] = 0.0f;
    jacobian[4] = 0.0f;
    jacobian[5] = -1.0f;
}

float angleViolation(AngleConstraint constraint, vec6 jacobian, float dt) {
    return -elc_vec6_dot(jacobian, V2FV2F_TO_V6(constraint.particle_a->velocity, constraint.particle_a->omega, constraint.particle_b->velocity, constraint.particle_b->omega)) - angleConstraint(constraint) / dt;
}

void applyAngleConstraint(AngleConstraint constraint, float dt) {
    vec6 jacobian;
    mat6 mass_matrix;
    createMassInertiaMatrix(constraint.particle_a->mass, constraint.particle_a->inertia, constraint.particle_b->mass, constraint.particle_b->inertia, mass_matrix);
    angleJacobian(jacobian);
    float den = inertiaImpulseDenominator(jacobian, mass_matrix);
    constraint.particle_a->omega += jacobian[2] * (angleViolation(constraint, jacobian, dt) / den);
    constraint.particle_a->omega += jacobian[5] * (angleViolation(constraint, jacobian, dt) / den);
}

float positionSpringStrength(PositionSpring spring) {
    return (fabsf(glm_vec2_distance(spring.position, spring.particle->position)) - spring.distance) * spring.stiffness;
}

void applyPositionSpring(PositionSpring spring) {
    vec2 direction;
    glm_vec2_sub(spring.position, spring.particle->position, direction);
    glm_vec2_normalize(direction);
    glm_vec2_muladds(direction, positionSpringStrength(spring), spring.particle->velocity);
}

void applyParticleGravity(Particle* particle, float dt /* delta time */) {
    particle->velocity[1] += 9.8f * dt; /* apply gravity * delta time */
}

void applyParticleVelocity(Particle* particle, float dt /* delta time */) {
    glm_vec2_muladds(particle->velocity, dt, particle->position);
    particle->rotation += particle->omega * dt;
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
    trail->points[trail->n_points++] = (Vector2){((float)800 / 2) + (point[0] * 75), ((float)600 / 2) + (point[1] * 75)};
}

void drawParticleTrail(ParticleTrail trail) {
    DrawLineStrip(trail.points, trail.n_points, WHITE);
}

void drawParticle(Particle particle, Color color) {
    Vector2 pos = {((float)800 / 2) + (particle.position[0] * 75), ((float)600 / 2) + (particle.position[1] * 75)};
    DrawCircleV(pos, 10, color);
    vec2 rotation_indicator = {6.0f};
    glm_vec2_rotate(rotation_indicator, particle.rotation, rotation_indicator);
    glm_vec2_add((float*)&pos, rotation_indicator, (float*)&pos);
    DrawCircleV(pos, 3, BLACK);
}

int main() {
    InitWindow(800, 600, "constraint solver");
    SetTargetFPS(240);

    Particle particle_a = {.position = {1.0f}, .mass = 1.0f, .inertia = 1.0f};
    Particle particle_b = {.position = {2.0f}, .mass = 1.0f, .inertia = 1.0f};
    Particle particle_c = {.position = {3.0f}, .mass = 1.0f, .inertia = 1.0f, .omega = 1.0f};
    OriginConstraint origin_constraint = {.distance = 1.0f, .particle = &particle_a};
    DistanceConstraint distance_constraint_a = {.distance = 1.0f, .particle_a = &particle_a, .particle_b = &particle_b};
    DistanceConstraint distance_constraint_b = {.distance = 1.0f, .particle_a = &particle_b, .particle_b = &particle_c};
    AngleConstraint angle_constraint_a = {.particle_a = &particle_b, .particle_b = &particle_c};
    PositionSpring spring = {.stiffness = 1.0f};

    ParticleTrail trail = createParticleTrail(ELC_KILOBYTE);

    while (!WindowShouldClose()) {
        float dt = (1.0f / 240.0f) / 1.0f;

        particleTrailAddPoint(&trail, particle_c.position);

        for (u32 i = 0; i < 1; i++) {
            applyParticleGravity(&particle_a, dt);
            applyParticleGravity(&particle_b, dt);
            applyParticleGravity(&particle_c, dt);

            if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
                Vector2 mouse = GetMousePosition();
                glm_vec2_copy((float*)&mouse, spring.position);
                glm_vec2_sub(spring.position, (vec2){800.0f / 2, 600.0f / 2}, spring.position);
                glm_vec2_divs(spring.position, 75.0f, spring.position);
                spring.particle = &particle_c;
                glm_vec2_scale(spring.particle->velocity, 0.75f, spring.particle->velocity);
                applyPositionSpring(spring);
            }

            for (u32 j = 0; j < 20; j++) {
                applyOriginConstraint(origin_constraint, dt);
                applyDistanceConstraint(distance_constraint_a, dt);
                applyDistanceConstraint(distance_constraint_b, dt);
                applyAngleConstraint(angle_constraint_a, dt);
            }

            applyParticleVelocity(&particle_a, dt);
            applyParticleVelocity(&particle_b, dt);
            applyParticleVelocity(&particle_c, dt);
        }

        BeginDrawing();

        ClearBackground(BLACK);

        drawParticleTrail(trail);

        DrawCircle(800 / 2, 600 / 2, 10, YELLOW);
        drawParticle(particle_a, BLUE);
        drawParticle(particle_b, RED);
        drawParticle(particle_c, ORANGE);

        DrawFPS(10, 10);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
