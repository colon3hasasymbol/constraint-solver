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

int main() {
    return 0;
}
