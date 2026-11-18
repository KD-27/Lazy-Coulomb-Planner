#!/usr/bin/env python3
"""
Run R2 on one map with a per-query hang guard.

Usage: run_r2_safe.py <exe> <maps_dir> <results_dir> <name> <repeats> [timeout_s]

R2 (github.com/LaiYanKai/2D) does not return on some queries whose goal is unreachable.
It keeps searching. This driver runs the whole scenario file once under a timeout.
On a timeout it reads the log to find the query that hung, drops that query and tries
again until a pass finishes. It then runs the remaining queries <repeats> times and
writes <name>.R2.<i>.results with the original query order. A dropped query is a
line holding only "timeout". Dropped query ids go to <name>.timeouts.txt.
"""
import os
import re
import shutil
import subprocess
import sys


def run(exe, work, res, name, n, timeout, log):
    with open(log, 'w') as f:
        try:
            subprocess.run([exe, '--map_dir', work, '--result_dir', res, '--names', name,
                            '--algs', 'R2', '--num_expts', str(n)],
                           stdout=f, stderr=subprocess.STDOUT, timeout=timeout, check=True)
            return True
        except subprocess.TimeoutExpired:
            return False


def main():
    exe, maps, results, name, repeats = sys.argv[1:6]
    repeats = int(repeats)
    timeout = float(sys.argv[6]) if len(sys.argv) > 6 else 60.0
    with open(os.path.join(maps, name + '.map.scen')) as f:
        header, *queries = f.read().splitlines()
    work = os.path.join(results, '_work', name)
    tmp_res = os.path.join(work, 'res')
    os.makedirs(tmp_res, exist_ok=True)
    link = os.path.join(work, name + '.map')
    if not os.path.exists(link):
        os.symlink(os.path.realpath(os.path.join(maps, name + '.map')), link)
    dropped = []
    log = os.path.join(results, name + '.log')
    while True:
        active = [i for i in range(len(queries)) if i not in dropped]
        with open(os.path.join(work, name + '.map.scen'), 'w') as f:
            f.write('\n'.join([header] + [queries[i] for i in active]) + '\n')
        if run(exe, work, tmp_res, name, 1, timeout, log):
            break
        done = [int(m) for m in re.findall(r'\[\s*(\d+)\s*/\s*\d+\s*\]', open(log).read())]
        hung = active[max(done) + 1 if done else 0]
        dropped.append(hung)
        print(f'{name}: query {hung} did not return within {timeout:.0f} s, dropped', flush=True)
    if repeats > 1 and not run(exe, work, tmp_res, name, repeats, timeout * repeats, log):
        sys.exit(f'{name}: timed run did not finish')
    for i in range(repeats):
        src = os.path.join(tmp_res, f'{name}.R2{"." + str(i) if repeats > 1 else ""}.results')
        rows = iter(open(src).read().splitlines())
        out = [('timeout' if q in dropped else next(rows)) for q in range(len(queries))]
        with open(os.path.join(results, f'{name}.R2.{i}.results'), 'w') as f:
            f.write('\n'.join(out) + '\n')
    with open(os.path.join(results, name + '.timeouts.txt'), 'w') as f:
        f.write(''.join(f'{q}\n' for q in sorted(dropped)))
    shutil.rmtree(work)


if __name__ == '__main__':
    main()
