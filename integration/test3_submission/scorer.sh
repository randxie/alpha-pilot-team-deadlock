#!/bin/bash
mkdir results
for i in `seq 0 24`;
do 
    f=$(rospack find flightgoggles)/config/challenges/challenge_final.yaml
    d=$(pwd)
    sed -i "7s@.*@results_location: \"$d\/results_gate_locations_$i.yaml\"@" $f 
    roslaunch flightgoggles scorer.launch level:=final gate_locations:=$i
    mv results_gate_locations_$i.yaml results
done

python scorer.py results
