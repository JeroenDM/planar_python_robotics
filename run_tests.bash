# run all .py scripts in this folder, and pytest
rm -rf __pycache
echo "========================================="
echo "Running tests"
echo "========================================="
PYTHONPATH=. pytest

#echo "========================================="
#echo "Running python scripts in this folder"
#echo "========================================="
#for s in *.py
#do
#  python3 $s
#done

echo "========================================="
echo "Running scripts form module ppr"
echo "========================================="
cd ppr
for m in *.py
do
 python3 $m
done
