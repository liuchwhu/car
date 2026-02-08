.PHONY: test lint format install manual wander calibrate

install:
	pip install -e ".[dev]"

test:
	pytest tests/ -v

lint:
	ruff check src/ tests/ scripts/

format:
	ruff format src/ tests/ scripts/

manual:
	pilotnano --mode manual --hardware mock

wander:
	pilotnano --mode wander --hardware mock

calibrate:
	python scripts/calibrate.py
