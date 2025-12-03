#include <cglm/io.h>
#include <cglm/mat3.h>
#include <cglm/mat4.h>
#include <cglm/quat.h>
#include <cglm/types.h>
#include <cglm/util.h>
#include <cglm/vec2.h>
#include <cglm/vec3.h>
#include <complex.h>
#include <math.h>
#include <raylib.h>
#include <elc/core.h>

typedef struct Particle {
    vec3 position;
    vec3 velocity;
    float mass;
    versor rotation;
    vec3 omega;
    mat3 inertia;
} Particle;

typedef struct BallJointConstraint {
    Particle* particle_a;
    Particle* particle_b;
    vec3 anchor_a, anchor_b;
} BallJointConstraint;

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

void createSphereInertiaTensor(float radius, mat3 dest) {
    glm_mat3_zero(dest);
    dest[0][0] = 1.0f / radius;
    dest[1][1] = 1.0f / radius;
    dest[2][2] = 1.0f / radius;
}

void createMassInertiaMatrix(float m_a, mat3 i_a, float m_b, mat3 i_b, mat12 dest) {
    elc_mat12_zero(dest);

    dest[0][0] = m_a;
    dest[1][1] = m_a;
    dest[2][2] = m_a;

    for (u8 i = 0; i < 3; i++) for (u8 j = 0; j < 3; j++) dest[i + 3][j + 3] = i_a[i][j];

    dest[6][6] = m_b;
    dest[7][7] = m_b;
    dest[8][8] = m_b;

    for (u8 i = 0; i < 3; i++) for (u8 j = 0; j < 3; j++) dest[i + 9][j + 9] = i_a[i][j];
}

void createMassMatrix(float m_a, float m_b, mat6 dest) {
    elc_mat6_zero(dest);
    dest[0][0] = m_a;
    dest[1][1] = m_a;
    dest[2][2] = m_a;
    dest[3][3] = m_b;
    dest[4][4] = m_b;
    dest[5][5] = m_b;
}

float originConstraint(OriginConstraint constraint) {
    return fabsf(glm_vec3_norm(constraint.particle->position)) - constraint.distance;
}

void originJacobian(vec3 point, vec3 jacobian) {
    float norm = fabsf(glm_vec3_norm(point));
    jacobian[0] = point[0] / norm;
    jacobian[1] = point[1] / norm;
    jacobian[2] = point[2] / norm;
}

float originViolation(OriginConstraint constraint, vec3 jacobian, float dt) {
    return -glm_vec3_dot(jacobian, constraint.particle->velocity) - originConstraint(constraint) / dt;
}

void applyOriginConstraint(OriginConstraint constraint, float dt) {
    vec3 jacobian;
    originJacobian(constraint.particle->position, jacobian);
    glm_vec3_muladds(jacobian, originViolation(constraint, jacobian, dt) / glm_vec3_dot(jacobian, jacobian), constraint.particle->velocity);
}

float ballJointConstraint(BallJointConstraint constraint) {
    vec3 anchor_a, anchor_b;
    glm_quat_rotatev(constraint.particle_a->rotation, constraint.anchor_a, anchor_a);
    glm_vec3_add(anchor_a, constraint.particle_a->position, anchor_a);
    glm_quat_rotatev(constraint.particle_b->rotation, constraint.anchor_b, anchor_b);
    glm_vec3_add(anchor_b, constraint.particle_b->position, anchor_b);
    return fabsf(glm_vec3_distance(anchor_a, anchor_b));
}

void ballJointJacobian(BallJointConstraint constraint, vec12 jacobian) {
    vec3 anchor_a, anchor_b, world_anchor_a, world_anchor_b, difference, cross_a, cross_b;
    glm_quat_rotatev(constraint.particle_a->rotation, constraint.anchor_a, anchor_a);
    glm_quat_rotatev(constraint.particle_b->rotation, constraint.anchor_b, anchor_b);
    glm_vec3_add(anchor_a, constraint.particle_a->position, world_anchor_a);
    glm_vec3_add(anchor_b, constraint.particle_b->position, world_anchor_b);
    glm_vec3_sub(world_anchor_b, world_anchor_a, difference);
    glm_vec3_cross(anchor_a, difference, cross_a);
    glm_vec3_cross(anchor_b, difference, cross_b);
    float norm = fabsf(glm_vec3_distance(world_anchor_a, world_anchor_b));

    jacobian[0] = -(difference[0] / norm);
    jacobian[1] = -(difference[1] / norm);
    jacobian[2] = -(difference[2] / norm);

    jacobian[3] = -(cross_a[0] / norm);
    jacobian[4] = -(cross_a[1] / norm);
    jacobian[5] = -(cross_a[2] / norm);

    jacobian[6] = difference[0] / norm;
    jacobian[7] = difference[1] / norm;
    jacobian[8] = difference[2] / norm;

    jacobian[9] = cross_b[0] / norm;
    jacobian[10] = cross_b[1] / norm;
    jacobian[11] = cross_b[2] / norm;
}

float massImpulseDenominator(vec6 jacobian, mat6 mass) {
    vec6 jtm;
    elc_mat6_mulv(mass, jacobian, jtm);
    return elc_vec6_dot(jtm, jacobian);
}

float inertiaImpulseDenominator(vec12 jacobian, mat12 mass) {
    vec12 jtm;
    elc_mat12_mulv(mass, jacobian, jtm);
    return elc_vec12_dot(jtm, jacobian);
}

float ballJointViolation(BallJointConstraint constraint, vec12 jacobian, float dt) {
    return -elc_vec12_dot(jacobian, V3V3V3V3_TO_V12(constraint.particle_a->velocity, constraint.particle_a->omega, constraint.particle_b->velocity, constraint.particle_b->omega)) - ballJointConstraint(constraint) / dt;
}

void applyBallJointConstraint(BallJointConstraint constraint, float dt) {
    vec12 jacobian;
    mat12 mass_matrix;
    createMassInertiaMatrix(constraint.particle_a->mass, constraint.particle_a->inertia, constraint.particle_b->mass, constraint.particle_b->inertia, mass_matrix);
    ballJointJacobian(constraint, jacobian);
    float den = inertiaImpulseDenominator(jacobian, mass_matrix);
    glm_vec3_muladds(&jacobian[0], ballJointViolation(constraint, jacobian, dt) / den, constraint.particle_a->velocity);
    glm_vec3_muladds(&jacobian[3], ballJointViolation(constraint, jacobian, dt) / den, constraint.particle_a->omega);
    glm_vec3_muladds(&jacobian[6], ballJointViolation(constraint, jacobian, dt) / den, constraint.particle_b->velocity);
    glm_vec3_muladds(&jacobian[9], ballJointViolation(constraint, jacobian, dt) / den, constraint.particle_b->omega);
}

float distanceConstraint(DistanceConstraint constraint) {
    return fabsf(glm_vec3_distance(constraint.particle_a->position, constraint.particle_b->position)) - constraint.distance;
}

void distanceJacobian(vec3 point_a, vec3 point_b, vec6 jacobian) {
    float norm = fabsf(glm_vec3_distance(point_a, point_b));
    jacobian[0] = -((point_b[0] - point_a[0]) / norm);
    jacobian[1] = -((point_b[1] - point_a[1]) / norm);
    jacobian[2] = -((point_b[2] - point_a[2]) / norm);
    jacobian[3] = (point_b[0] - point_a[0]) / norm;
    jacobian[4] = (point_b[1] - point_a[1]) / norm;
    jacobian[5] = (point_b[2] - point_a[2]) / norm;
}

float distanceViolation(DistanceConstraint constraint, vec6 jacobian, float dt) {
    return -elc_vec6_dot(jacobian, V3V3_TO_V6(constraint.particle_a->velocity, constraint.particle_b->velocity)) - distanceConstraint(constraint) / dt;
}

void applyDistanceConstraint(DistanceConstraint constraint, float dt) {
    vec6 jacobian;
    mat6 mass_matrix;
    createMassMatrix(constraint.particle_a->mass, constraint.particle_b->mass, mass_matrix);
    distanceJacobian(constraint.particle_a->position, constraint.particle_b->position, jacobian);
    float den = massImpulseDenominator(jacobian, mass_matrix);
    glm_vec3_muladds(&jacobian[0], distanceViolation(constraint, jacobian, dt) / den, constraint.particle_a->velocity);
    glm_vec3_muladds(&jacobian[3], distanceViolation(constraint, jacobian, dt) / den, constraint.particle_b->velocity);
}

void applyParticleGravity(Particle* particle, float dt) {
    particle->velocity[1] -= 9.8f * dt;
}

void applyParticleVelocity(Particle* particle, float dt) {
    glm_vec3_muladds(particle->velocity, dt, particle->position);

    vec3 alpha;
    glm_mat3_mulv(particle->inertia, particle->omega, alpha);
    glm_vec3_cross(particle->omega, alpha, alpha);
    glm_mat3_inv(particle->inertia, particle->inertia);
    glm_mat3_mulv(particle->inertia, alpha, alpha);
    glm_vec3_muladds(alpha, dt, particle->omega);

    float angle = glm_vec3_norm(particle->omega);
    vec3 axis;
    glm_vec3_normalize_to(particle->omega, axis);
    versor rotation;
    glm_quatv(rotation, angle * 0.5f * dt, axis);
    glm_quat_mul(particle->rotation, rotation, particle->rotation);
    glm_quat_normalize(particle->rotation);
}

Matrix particleTransform(Particle particle) {
    mat4 transform, rotation;
    glm_quat_mat4(particle.rotation, rotation);
    glm_scale_make(transform, (vec3){1.0f, 1.0f, 1.0f});
    glm_translate(transform, particle.position);
    glm_mat4_mul(transform, rotation, transform);
    glm_mat4_transpose(transform);
    return *((Matrix*)&transform);
}

void drawBallJointConstraint(BallJointConstraint constraint, Color color) {
    vec3 world_anchor_a;
    glm_quat_rotatev(constraint.particle_a->rotation, constraint.anchor_a, world_anchor_a);
    glm_vec3_add(world_anchor_a, constraint.particle_a->position, world_anchor_a);
    DrawSphere((Vector3)VEC3_USE(world_anchor_a), 0.25f, color);
}

int main() {
    InitWindow(800, 600, "constraint solver");
    SetTargetFPS(240);

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 40.0f, 40.0f, 40.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Mesh cube_mesh = GenMeshCube(1.0f, 1.0f, 1.0f);
    Model cube_model = LoadModelFromMesh(cube_mesh);

    Particle particle_a = {.mass = 1.0f, .position = {5.0f}, .inertia = GLM_MAT3_IDENTITY_INIT};
    Particle particle_b = {.mass = 1.0f, .position = {10.0f}, .inertia = GLM_MAT3_IDENTITY_INIT};
    OriginConstraint origin_constraint_a = {.particle = &particle_a, .distance = 5.0f};
    BallJointConstraint distance_constraint_a = {.particle_a = &particle_a, .particle_b = &particle_b, .anchor_a = {2.5f}, .anchor_b = {2.5f}};

    while (!WindowShouldClose()) {
        float dt = (1.0f / 240.0f) / 1.0f;

        applyParticleGravity(&particle_a, dt);
        applyParticleGravity(&particle_b, dt);

        for (u32 i = 0; i < 20; i++) {
            applyOriginConstraint(origin_constraint_a, dt);
            applyBallJointConstraint(distance_constraint_a, dt);
        }

        applyParticleVelocity(&particle_a, dt);
        applyParticleVelocity(&particle_b, dt);

        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);

        DrawCube((Vector3){0}, 1.0f, 1.0f, 1.0f, YELLOW);
        cube_model.transform = particleTransform(particle_a);
        DrawModel(cube_model, (Vector3){0}, 1.0f, BLUE);
        cube_model.transform = particleTransform(particle_b);
        DrawModel(cube_model, (Vector3){0}, 1.0f, RED);
        drawBallJointConstraint(distance_constraint_a, ORANGE);

        EndMode3D();
        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
