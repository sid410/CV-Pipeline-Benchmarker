#!/bin/bash

# hide imshow
for p in {0..3}; do
    for b in {0..3}; do
        for t in 1 2 4 8 16; do

        python3.11 benchmark_cpu.py "-p $p" "-b $b" "-t $t" -mc 1000 -f
        pid=$!
        wait $pid
        sleep 3

        done
    done
done

# show imshow
for p in {0..3}; do
    for b in {0..3}; do
        for t in 1 2 4 8 16; do

        python3.11 benchmark_cpu.py "-p $p" "-b $b" "-t $t" -mc 1000
        pid=$!
        wait $pid
        sleep 3

        done
    done
done

sudo shutdown -h now