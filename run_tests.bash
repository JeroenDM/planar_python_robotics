# run all .py scripts in this folder, and pytest
#echo "========================================="
#rm -rf __pycache
#echo "Running pytest"
#echo "========================================="
#pytest
echo "========================================="
echo "Running python scripts in this folder"
echo "========================================="
for s in *.py
do
  python3 $s
done

echo "========================================="
echo "Running scripts form module ppr"
echo "========================================="
for m in ppr/*.py
do
 python3 $m
done
