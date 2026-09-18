#pragma once

#include <cstddef>
#include <cstdint>

#ifdef _WIN32
#define PYROBO_API __declspec(dllexport)
#else
#define PYROBO_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

using pyrobo_point = struct pyrobo_point {
    double x;
    double y;
};

using pyrobo_callbacks = struct pyrobo_callbacks {
    void* user_data;

    int (*get_map)(
        void* user_data, int* height, int* width, double* resolution,
        double* origin_x, double* origin_y, const uint8_t** data,
        size_t* data_size
    );
    int (*get_pose)(
        void* user_data, const char* robot_id, double* x, double* y, double* yaw
    );
    int (*get_robot)(void* user_data, const char* robot_id, double* radius);
    int (*get_goal)(void* user_data, double* x, double* y, double* yaw);
    int (*get_feedback)(
        void* user_data, const char* robot_id, double* first, double* second
    );
    int (*set_control)(
        void* user_data, const char* robot_id, double first, double second
    );
    int (*get_control)(
        void* user_data, const char* robot_id, double* first, double* second
    );
    int (*get_lidar)(
        void* user_data, const char* robot_id, int count, double max_range,
        double fov, double* output, size_t capacity, size_t* output_size
    );
    int (*set_display_path)(
        void* user_data, const pyrobo_point* points, size_t count
    );
    int (*set_planning_map)(
        void* user_data, int height, int width, double resolution,
        double origin_x, double origin_y, const uint8_t* data, size_t data_size
    );
};

PYROBO_API void* pyrobo_create_navigation(
    const pyrobo_callbacks* callbacks, char* error_message,
    size_t error_capacity
);

PYROBO_API int pyrobo_run_navigation(
    void* navigation, char* error_message, size_t error_capacity
);

PYROBO_API void pyrobo_destroy_navigation(void* navigation);

#ifdef __cplusplus
}
#endif
