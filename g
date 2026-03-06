import time

N = 100_000_000
s = 0.0

start = time.perf_counter()

for i in range(N):
    s += i * 0.000001

elapsed_ms = (time.perf_counter() - start) * 1000

print(f"Sum: {s}")
print(f"Elapsed: {elapsed_ms:.2f} ms")