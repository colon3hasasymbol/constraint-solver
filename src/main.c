#include <cglm/vec2.h>
#include <raylib.h>
#include <elc/core.h>

typedef struct HeapMatrix {
    float* data;
    u32 width, height;
} HeapMatrix;

void createMatrix(u32 width, u32 height, HeapMatrix* matrix) {
    *matrix = (HeapMatrix){.width = width, .height = height, .data = malloc(width * height * sizeof(float))};
}

void destroyMatrix(HeapMatrix* matrix) {
    free(matrix->data);
}

float indexMatrix(HeapMatrix* matrix, u32 x, u32 y) {
    return matrix->data[x + (y * matrix->width)];
}

void setMatrix(HeapMatrix* matrix, u32 x, u32 y, float value) {
    matrix->data[x + (y * matrix->width)] = value;
}

void multiplyMatrix(HeapMatrix* matrix_a, HeapMatrix* matrix_b, HeapMatrix* dest) {
    createMatrix(matrix_b->width, matrix_a->height, dest);

    for (u32 i = 0; i < matrix_a->height; i++)
        for (u32 j = 0; j < matrix_b->width; j++) {
            float v = 0.0f;
            for (u32 k = 0; k < matrix_a->width; k++) v += indexMatrix(matrix_a, i, k) * indexMatrix(matrix_b, k, j);

            setMatrix(dest, i, j, v);
        }
}

void componentMultiplyMatrix(HeapMatrix* matrix_a, HeapMatrix* matrix_b, HeapMatrix* dest) {
    createMatrix(matrix_a->width, matrix_b->width, dest);

    for (u32 i = 0; i < matrix_a->width; i++)
        for (u32 j = 0; j < matrix_a->height; j++)
            setMatrix(dest, i, j, indexMatrix(matrix_a, i, j) * indexMatrix(matrix_b, i, j));
}

void transposeMultiplyMatrix(HeapMatrix* matrix_a, HeapMatrix* matrix_b, HeapMatrix* dest) {
    createMatrix(matrix_b->width, matrix_a->width, dest);

    for (u32 i = 0; i < matrix_a->width; i++)
        for (u32 j = 0; j < matrix_b->width; j++) {
            float v = 0.0f;
            for (u32 k = 0; k < matrix_a->height; k++) v += indexMatrix(matrix_a, k, i) * indexMatrix(matrix_b, k, j);

            setMatrix(dest, i, j, v);
        }
}

void transposeMatrix(HeapMatrix* matrix, HeapMatrix* dest) {
    createMatrix(matrix->height, matrix->width, dest);

    for (u32 i = 0; i < matrix->width; i++)
        for (u32 j = 0; j < matrix->height; j++)
            setMatrix(dest, j, i, indexMatrix(matrix, i, j));
}

void subtractMatrix(HeapMatrix* matrix_a, HeapMatrix* matrix_b, HeapMatrix* dest) {
    createMatrix(matrix_a->width, matrix_a->height, dest);

    for (u32 i = 0; i < matrix_a->width; i++)
        for (u32 j = 0; j < matrix_a->height; j++)
            setMatrix(dest, i, j, indexMatrix(matrix_a, i, j) - indexMatrix(matrix_b, i, j));
}

void negateMatrix(HeapMatrix* matrix, HeapMatrix* dest) {
    createMatrix(matrix->width, matrix->height, dest);

    for (u32 i = 0; i < matrix->width; i++)
        for (u32 j = 0; j < matrix->height; j++)
            setMatrix(dest, i, j, -indexMatrix(matrix, i, j));
}

typedef struct Particle {
    vec2 position;
    vec2 velocity;
} Particle;

float pointDistanceConstraint(vec2 point, float distance) {
    return fabsf(glm_vec2_norm(point)) - distance;
}

void pointDistanceJacobian(vec2 point, vec2 jacobian /* using `vec2` as a `mat1x2` */) {
    float norm = fabsf(glm_vec2_norm(point)); /* length of `point` */
    jacobian[0] = point[0] / norm; /* x / sqrt(x^2 + y^2) */
    jacobian[1] = point[1] / norm; /* y / sqrt(x^2 + y^2) */
}

float pointDistanceViolation(Particle particle, vec2 jacobian, float dt) {
    return -glm_vec2_dot(jacobian, particle.velocity) - pointDistanceConstraint(particle.position, 1.0f) / dt;
}

void particleApplyGravity(Particle* particle, float dt /* delta time */) {
    particle->velocity[1] += 9.8f * dt; /* apply gravity * delta time */
}

void particleApplyConstraint(Particle* particle, float dt) {
    vec2 jacobian;
    pointDistanceJacobian(particle->position, jacobian);
    glm_vec2_muladds(jacobian, pointDistanceViolation(*particle, jacobian, dt) / glm_vec2_dot(jacobian, jacobian), particle->velocity);
}

void applyParticleVelocity(Particle* particle, float dt /* delta time */) {
    glm_vec2_muladds(particle->velocity, dt, particle->position);
}

void updateParticle(Particle* particle, float dt) {
    particleApplyGravity(particle, dt);
    particleApplyConstraint(particle, dt);
    applyParticleVelocity(particle, dt);
}

int main() {
    InitWindow(800, 600, "constraint solver");
    SetTargetFPS(60);

    Particle particle = {.position = {1.0f}};

    while (!WindowShouldClose()) {
        updateParticle(&particle, 1.0f / 60.0f);

        BeginDrawing();

        ClearBackground(BLACK);

        DrawCircle(800 / 2, 600 / 2, 5, YELLOW);
        DrawCircle(((float)800 / 2) + (particle.position[0] * 50), ((float)600 / 2) + (particle.position[1] * 50), 5, BLUE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
