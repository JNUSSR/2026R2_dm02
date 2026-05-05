
#include "Cylinder.h"


void Cylinder_Init() {
    // Default to pull-back to avoid unexpected cylinder motion on boot.
    HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT1, Cylinder_CYLINDER_GPIO_PIN1, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT2, Cylinder_CYLINDER_GPIO_PIN2, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(Cylinder_CYLINDER_POWER_GPIO_PORT, Cylinder_CYLINDER_POWER_GPIO_PIN, GPIO_PIN_SET);
}

void Cylinder1_Push() {
    HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT1, Cylinder_CYLINDER_GPIO_PIN1, GPIO_PIN_SET);
}

// void Cylinder2_Push() {
//     HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT2, Cylinder_CYLINDER_GPIO_PIN2, GPIO_PIN_SET);
// }

void Cylinder_Push() {
    Cylinder1_Push();
    // Cylinder2_Push();
}

void Cylinder_Pull() {
    HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT1, Cylinder_CYLINDER_GPIO_PIN1, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT2, Cylinder_CYLINDER_GPIO_PIN2, GPIO_PIN_RESET);
}

void Cylinder1_Pull() {
    HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT1, Cylinder_CYLINDER_GPIO_PIN1, GPIO_PIN_RESET);
}

// void Cylinder2_Pull() {
//     HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT2, Cylinder_CYLINDER_GPIO_PIN2, GPIO_PIN_RESET);
// }