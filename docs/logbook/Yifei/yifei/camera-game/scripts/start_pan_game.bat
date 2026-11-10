@echo off
cd /d "%~dp0.."
python scripts\pan_game.py
if errorlevel 1 pause
