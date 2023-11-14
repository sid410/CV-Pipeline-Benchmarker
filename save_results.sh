#!/bin/bash

for p in {0..3}; do
    for b in {0..3}; do
        for t in 1 2 4 8 16; do

        python3.11 benchmark_cpu.py "-p $p" "-b $b" "-t $t" -mc 300 -f
        pid=$!
        wait $pid
        sleep 3

        done
    done
done
