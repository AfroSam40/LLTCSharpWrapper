import pyperf

N = 20_000_000

def bench():
    x = [0.0] * N
    y = [0.0] * N
    z = [0.0] * N
    for i in range(N):
        x[i] = x[i] * 1.0001 + 0.01
        y[i] = y[i] * 0.9999 - 0.02
        z[i] = z[i] * 1.0000 + 0.03

runner = pyperf.Runner()
runner.bench_func("for_loop", bench)