"""Configuration loader using OmegaConf."""

from __future__ import annotations

from pathlib import Path
from omegaconf import OmegaConf, DictConfig

CONFIGS_DIR = Path(__file__).resolve().parents[2] / "configs"


def load_config(
    config_path: str | None = None,
    hardware_override: str | None = None,
    cli_overrides: list[str] | None = None,
) -> DictConfig:
    """Load and merge configuration.

    Priority (highest wins): CLI overrides > hardware override > default.yaml
    """
    base = OmegaConf.load(CONFIGS_DIR / "default.yaml")

    # Merge referenced sub-configs
    for key in ("hardware", "behavior", "calibration"):
        ref = base.get(key)
        if isinstance(ref, str):
            sub_path = CONFIGS_DIR / ref
            if sub_path.exists():
                sub = OmegaConf.load(sub_path)
                base[key] = sub

    # Hardware override (e.g., --hardware mock selects hardware/mock.yaml)
    if hardware_override:
        hw_path = CONFIGS_DIR / "hardware" / f"{hardware_override}.yaml"
        if hw_path.exists():
            hw = OmegaConf.load(hw_path)
            base.hardware = OmegaConf.merge(base.hardware, hw)

    # Optional extra config file
    if config_path:
        extra = OmegaConf.load(config_path)
        base = OmegaConf.merge(base, extra)

    # CLI dot-notation overrides (e.g., hardware.actuator.steering_trim=0.05)
    if cli_overrides:
        cli = OmegaConf.from_dotlist(cli_overrides)
        base = OmegaConf.merge(base, cli)

    return base
