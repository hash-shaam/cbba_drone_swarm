import csv
import matplotlib.pyplot as plt

drones = ["drone1", "drone2", "drone3"]
colors = {"drone1": "tab:blue", "drone2": "tab:red", "drone3": "tab:green"}

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

for d in drones:
    times, xs, ys, mins = [], [], [], []
    with open(f"/tmp/{d}_positions.csv") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            m = row["min_distance"]
            mins.append(float(m) if m != "inf" else None)

    t0 = times[0]
    times = [t - t0 for t in times]

    ax1.plot(xs, ys, label=d, color=colors[d])
    ax1.scatter(xs[0], ys[0], color=colors[d], marker="o")   # start
    ax1.scatter(xs[-1], ys[-1], color=colors[d], marker="x")  # end

    ax2.plot(times, mins, label=d, color=colors[d])

ax1.set_title("Trajectories (o=start, x=end)")
ax1.set_xlabel("x (m)")
ax1.set_ylabel("y (m)")
ax1.legend()
ax1.axis("equal")

ax2.axhline(2.0, color="orange", linestyle="--", label="soft threshold")
ax2.axhline(0.5, color="red", linestyle="--", label="hard threshold")
ax2.set_title("Min pairwise distance over time")
ax2.set_xlabel("time (s)")
ax2.set_ylabel("distance (m)")
ax2.legend()

plt.tight_layout()
plt.savefig("/tmp/swarm_result.png", dpi=150)
plt.show()
print("Saved to /tmp/swarm_result.png")
