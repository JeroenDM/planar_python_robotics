rm -rf __pycache
echo "Running pytest"
echo "--------------"
pytest
echo "Running main.py"
echo "---------------"
python3 main.py
