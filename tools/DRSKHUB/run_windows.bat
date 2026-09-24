@echo off
if not exist venv python -m venv venv
call venv\Scriptsctivate.bat
pip install -r requirements.txt -q
python main.py
pause
