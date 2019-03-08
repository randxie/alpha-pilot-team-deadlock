import os
import sys
import json

def read_file(file="results.txt"):
    with open(file, 'r') as f:
        x = [i.split() for i in f.readlines() if i.split() != []]
    return x

def parse(data,order='Ll,Lr,Ul,Ur',fullpath=False):
    result = {}
    for i in data:     
        d = i[0].split(',')
        d = [i.replace('(','').replace(')','') for i in d]
        ll,lr,ur,ul = [list(map(int,list(map(float,d[i:i+2])))) for i in range(4)]
        if fullpath:
            filename = d[-1]
        else:
            filename = d[-1].split('/')[-1]

        result[filename] = [ll+lr+ur+ul]
    return result

def write_json(dct, out='simulated_results_data.json'):
    with open('simulated_results_data.json','w') as f:
        f.write(json.dumps(dct))
    return 1

if __name__ == "__main__":
    x = read_file()
    results = parse(x)
    write_json(results) 
