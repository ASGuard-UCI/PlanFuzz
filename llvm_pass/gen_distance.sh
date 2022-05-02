echo "Computing distance ...."
for f in $(ls -1d ./dot_dir/cfg.*.dot); do
  if ! grep "$(basename $f | cut -d. -f2)" ./dot_dir/callgraph.dot >/dev/null; then
    printf "\nSkipping $f..\n"
    continue
  fi

  #awk '!a[$0]++' $f > ${f}.smaller.dot
  #mv $f $f.bigger.dot
  #mv $f.smaller.dot $f
  #sed -i s/\\\\\"//g $f
  #sed -i 's/\[.\"]//g' $f
  #sed -i 's/\(^\s*[0-9a-zA-Z_]*\):[a-zA-Z0-9]*\( -> \)/\1\2/g' $f

  #Compute distance
  printf "\nComputing distance for $f..\n"
  python distance.py -d $f -t ./target.txt -n ./BBnames.txt -s ./BBcalls.txt -c ./distance.callgraph.txt -o ${f}.distances.txt 
  if [ $? -ne 0 ]; then
    echo -e "\e[93;1m[!]\e[0m Could not calculate distance for $f."
  fi
done
