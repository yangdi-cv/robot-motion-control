"""Atomic status and one-second trace chunks for interrupted long runs."""

import json
import os
from pathlib import Path
import numpy as np


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix(".tmp")
    with temp.open("w") as f:
        json.dump(value, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    temp.replace(path)


def chunk(directory, index, rows):
    directory = Path(directory)
    directory.mkdir(exist_ok=True)
    path = directory / f"{index:06d}.npz"
    temp = path.with_suffix(".tmp")
    with temp.open("wb") as f:
        np.savez_compressed(
            f, **{key: np.array([r[key] for r in rows]) for key in rows[0]}
        )
        f.flush()
        os.fsync(f.fileno())
    temp.replace(path)
