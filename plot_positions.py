import csv
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

drones = ["drone1", "drone2", "drone3"]
colors = {"drone1": "tab:blue", "drone2": "tab:red", "drone3": "tab:green"}

fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(13, 9))

for d in drones:
    times, xs, ys, mins, align_errs, coh_errs = [], [], [], [], [], []
    with open(f"/tmp/{d}_positions.csv") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            m = row["min_distance"]
            mins.append(float(m) if m != "inf" else None)
            align_errs.append(float(row.get("alignment_error_deg", 0.0)))
            coh_errs.append(float(row.get("cohesion_error_m", 0.0)))

    t0 = times[0]
    times = [t - t0 for t in times]

    ax1.plot(xs, ys, label=d, color=colors[d])
    ax1.scatter(xs[0], ys[0], color=colors[d], marker="o")
    ax1.scatter(xs[-1], ys[-1], color=colors[d], marker="x")

    ax2.plot(times, mins, label=d, color=colors[d])
    ax3.plot(times, align_errs, label=d, color=colors[d])
    ax4.plot(times, coh_errs, label=d, color=colors[d])

    try:
        with open(f"/tmp/{d}_events.csv") as f:
            reader = csv.DictReader(f)
            for row in reader:
                ax1.scatter(float(row["x"]), float(row["y"]),
                            color=colors[d], marker="D", s=90,
                            edgecolor="black", linewidth=1.0, zorder=5)
    except FileNotFoundError:
        pass

try:
    with open("/tmp/tasks.csv") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tx, ty, tid = float(row["x"]), float(row["y"]), row["task_id"]
            ax1.scatter(tx, ty, color="black", marker="*", s=250, zorder=6)
            ax1.annotate(str(tid), (tx, ty), textcoords="offset points",
                         xytext=(8, 8), fontsize=13, fontweight="bold")
except FileNotFoundError:
    pass

task_handle = Line2D([0], [0], marker="*", color="w", markerfacecolor="black",
                      markersize=14, label="task (numbered)")
arrival_handle = Line2D([0], [0], marker="D", color="w", markerfacecolor="grey",
                         markeredgecolor="black", markersize=9, label="held within 3m")

ax1.set_title("Trajectories (o=start, x=end)")
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
handles, labels = ax1.get_legend_handles_labels()
ax1.legend(handles=handles + [task_handle],
           labels=labels + ["task (numbered)"])
ax1.axis("equal")

ax2.axhline(2.0, color="orange", linestyle="--", label="soft threshold")
ax2.axhline(0.5, color="red", linestyle="--", label="hard threshold")
ax2.set_title("Min pairwise distance over time")
ax2.set_xlabel("time (s)"); ax2.set_ylabel("distance (m)")
ax2.legend()

ax3.set_title("Alignment error (deg from mean neighbour heading)")
ax3.set_xlabel("time (s)"); ax3.set_ylabel("degrees")
ax3.legend()

ax4.set_title("Cohesion error (distance to neighbour centroid)")
ax4.set_xlabel("time (s)"); ax4.set_ylabel("metres")
ax4.legend()

plt.tight_layout()
plt.savefig("/tmp/swarm_result.png", dpi=150)
plt.show()
print("Saved to /tmp/swarm_result.png")