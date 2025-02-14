
perf record -g -o perf.data ./main.exe -i inputs/pianoroom.ray -png -o output/pianoroom.png -H 500 -W 500

perf script > out.perf

cd FlameGraph

./stackcollapse-perf.pl ../out.perf > out.folded

./flamegraph.pl out.folded > ../flamegraph.svg

echo "Flame graph generated: FlameGraph/flamegraph.svg"
