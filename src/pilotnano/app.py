"""PilotNano CLI entry point and main loop."""

import argparse
import logging
import time

from pilotnano.config import load_config
from pilotnano.utils.logging import setup_logging
from pilotnano.bus import LocalBus
from pilotnano.hardware import create_camera, create_actuator

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PilotNano — self-driving RC car")
    parser.add_argument("--mode", default="manual", choices=["manual", "wander", "calibrate"],
                        help="Driving mode (default: manual)")
    parser.add_argument("--hardware", default=None,
                        help="Hardware config override (e.g., 'mock' for desktop dev)")
    parser.add_argument("--config", default=None,
                        help="Path to additional config file to merge")
    parser.add_argument("--force-calibrate", action="store_true",
                        help="Force re-calibration even if saved results exist")
    parser.add_argument("overrides", nargs="*",
                        help="OmegaConf dot-notation overrides (e.g., loop_hz=30)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    cfg = load_config(
        config_path=args.config,
        hardware_override=args.hardware,
        cli_overrides=args.overrides if args.overrides else None,
    )

    setup_logging(cfg.get("log_level", "INFO"))
    logger.info("PilotNano starting — mode=%s", args.mode)

    bus = LocalBus()
    camera = create_camera(cfg.hardware)
    actuator = create_actuator(cfg.hardware)

    # Build behavior manager for autonomous modes
    behavior_manager = None
    if args.mode in ("wander", "follow"):
        from pilotnano.behavior.manager import BehaviorManager
        behavior_manager = BehaviorManager(cfg, bus)

    from pilotnano.modes import create_mode
    mode = create_mode(args.mode, cfg, bus, camera, actuator, behavior_manager)

    tick_duration = 1.0 / cfg.get("loop_hz", 20)

    with camera, actuator:
        mode.start()
        try:
            while mode.running:
                t0 = time.monotonic()
                mode.step()
                elapsed = time.monotonic() - t0
                sleep_time = tick_duration - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif elapsed > tick_duration * 1.5:
                    logger.warning("Loop overrun: %.1fms (budget: %.1fms)",
                                   elapsed * 1000, tick_duration * 1000)
        except KeyboardInterrupt:
            logger.info("Ctrl+C received, stopping...")
        finally:
            mode.stop()

    logger.info("PilotNano stopped")


if __name__ == "__main__":
    main()
