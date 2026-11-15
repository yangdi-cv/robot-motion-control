from pathlib import Path
from core import Config, simulate, CONTROLLERS

R = Path(__file__).resolve().parents[1]
for method in ["open_loop", *CONTROLLERS]:
    for impulse in [0, 6, 12]:
        cfg = Config(
            controller=method, impulse_x=impulse, noise_position=0.0, noise_velocity=0.0
        )
        result, _ = simulate(cfg, R / "runs" / f"pilot_{method}_{impulse}")
        print(
            method,
            impulse,
            result["success"],
            result["reason"],
            "recovery",
            result["recovery_s"],
            "route",
            round(result["terminal_route_error_m"], 3),
            "rmse",
            round(result["tracking_rmse_m"], 3),
            flush=True,
        )
