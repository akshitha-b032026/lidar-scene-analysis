@echo off
REM Usage: run.bat [C:\path\to\nuscenes]
cd /d "%~dp0"
if not exist .venv python -m venv .venv
call .venv\Scripts\activate
pip install -q -r requirements.txt
if not "%~1"=="" set NUSCENES_DATAROOT=%~1
uvicorn backend.app:app --reload
