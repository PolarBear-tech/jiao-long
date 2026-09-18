"""Configuration for the regular exercise map."""

CONFIG = {
    "meta": {
        "author": "Musheng-bot",
        "email": "meisoren081@gmail.com",
    },
    "pyrobo": {
        "robot_radius": 0.3,
        "time_step": 0.05,
        "map": {
            "origin_x": 0.0,
            "origin_y": 0.0,
            "resolution": 0.05,
            "name": "map",
        },
        "control": {
            "mode": "auto",
            "dynamics": {
                "vx_max": 1.5,
                "vx_min": 0.0,
                "vy_max": 1.5,
                "vy_min": 0.0,
                "acc_max": 1.0,
                "acc_min": -1.0,
            },
        },
        "robot": {
            "initial": {
                "x": 0.50,
                "y": 0.50,
                "yaw": 0.0,
            },
            "goal": {
                "x": 9.50,
                "y": 7.50,
                "yaw": 0.0,
            },
        },
    },
}
