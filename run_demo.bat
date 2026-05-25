@echo off
echo ============================================================
echo   LIFE-LINK Smart Traffic System — Demo Runner (Windows)
echo ============================================================

set MODE=%1
if "%MODE%"=="" set MODE=--headless

if "%MODE%"=="--test" (
    python -m pytest tests/ -v
) else if "%MODE%"=="--analysis" (
    python scripts/generate_charts.py
    python scripts/run_analysis.py
) else if "%MODE%"=="--headless" (
    python scripts/run_demo_headless.py
    python scripts/generate_sample_csv.py
    python scripts/generate_charts.py
) else if "%MODE%"=="--pygame" (
    python -m src.ui.pygame_ui
)
