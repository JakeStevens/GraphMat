#!/usr/bin/env python3

import re
import pandas
import subprocess
import sys

def run_and_get_breakdown(cmd):
  ret = subprocess.check_output(cmd.split()).decode(sys.stdout.encoding)
  pattern = 'compute Time = .*? \((?P<percent>[01]\.[0-9][0-9][0-9])'
  compute = re.search(pattern, ret)
  compute = compute.group('percent')
  pattern = 'graph Time = .*? \((?P<percent>[01]\.[0-9][0-9][0-9])'
  graph = re.search(pattern, ret)
  graph = graph.group('percent')
  return compute, graph

labels = ['Benchmarks', 'Compute', 'Graph']
benchmarks = []
computes = []
graphs = []

# Run BFS
cmd = 'mpirun -np 1 numactl -i all bin/BFS data/cora.mtx 4'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('BFS')
computes.append(compute)
graphs.append(graph)

# Run Delta Stepping
cmd = 'mpirun -np 1 numactl -i all bin/DeltaStepping data/cora.mtx 1 6'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('DELTA')
computes.append(compute)
graphs.append(graph)

# Run LDA
cmd = 'mpirun -np 1 numactl -i all bin/LDA data/cora.mtx 2000 708 100'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('LDA')
computes.append(compute)
graphs.append(graph)

# Run PageRank
cmd = 'mpirun -np 1 numactl -i all bin/PageRank data/cora.mtx'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('PR')
computes.append(compute)
graphs.append(graph)

# Run SGD
cmd = 'mpirun -np 1 numactl -i all bin/SGD data/cora.mtx'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('SGD')
computes.append(compute)
graphs.append(graph)

# Run SSSP
cmd = 'mpirun -np 1 numactl -i all bin/SSSP data/cora.mtx 6'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('SSSP')
computes.append(compute)
graphs.append(graph)

# Run Topological Sort
cmd = 'mpirun -np 1 numactl -i all bin/TopologicalSort data/cora.mtx'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('TOP')
computes.append(compute)
graphs.append(graph)

# Run Triangle Counting
cmd = 'mpirun -np 1 numactl -i all bin/TriangleCounting data/cora.mtx'
compute, graph = run_and_get_breakdown(cmd)
benchmarks.append('TRI')
computes.append(compute)
graphs.append(graph)

# Save as a csv
print(','.join(labels))
for entry in zip(benchmarks, computes, graphs):
  print(','.join(entry))
