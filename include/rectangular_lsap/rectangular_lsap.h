#ifndef RECTANGULAR_LSAP_H
#define RECTANGULAR_LSAP_H

#define RECTANGULAR_LSAP_INFEASIBLE -1
#define RECTANGULAR_LSAP_INVALID -2

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

int solve_rectangular_linear_sum_assignment(intptr_t nr, intptr_t nc, double* input_cost, bool maximize, int64_t* a,
                                            int64_t* b);

#ifdef __cplusplus
}
#endif

#endif