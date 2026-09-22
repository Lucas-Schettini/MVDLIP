import json
import sys
 
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import itertools, math 
 
def load_instance(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def dist(coords, i, j):
    xi, yi = coords[i]
    xj, yj = coords[j]
    return math.hypot(xi - xj, yi - yj) 
 
def plot_instance(data, out_path=None):
    coords = {int(k): (v["x"], v["y"]) for k, v in data["coordinates"].items()}
    D = set(data["sets"]["D"])
    P = set(data["sets"]["P"])
    depot_ids = P - D  

    fig, ax = plt.subplots(figsize=(9, 9))
 
    xs = [c[0] for c in coords.values()]
    ys = [c[1] for c in coords.values()]
    ax.scatter(xs, ys, s=8, color="lightgray", zorder=1)
 
    cmap = plt.get_cmap("tab10")
    lines_seen = sorted({e["line"] for e in data["required_edges"]})
    line_color = {lid: cmap(i % 10) for i, lid in enumerate(lines_seen)}
 
    for e in data["required_edges"]:
        u, v = e["u"], e["v"]
        if u not in coords or v not in coords:
            continue
        (x1, y1), (x2, y2) = coords[u], coords[v]
        ax.plot([x1, x2], [y1, y2],
                color=line_color[e["line"]], linewidth=2.2, zorder=2)
 
    # VrD = data["sets"]["VR"] + data["sets"]["D"]
    # Enr = [(i, j, dist(coords, i, j)) for i, j in itertools.combinations(VrD, 2)]

    # for er in Enr:
    #     u, v, d = er            
    #     if u not in coords or v not in coords:
    #         continue
    #     (x1, y1), (x2, y2) = coords[u], coords[v]
    #     ax.plot([x1, x2], [y1, y2],
    #             color="black", linewidth=0.5, alpha=0.15, zorder=0.5)

    for a in data["Ap"]:
        i, j = a["i"], a["j"]
        if i not in coords or j not in coords:
            continue
        (x1, y1), (x2, y2) = coords[i], coords[j]
        ax.plot([x1, x2], [y1, y2],
                color="black", linestyle="--", linewidth=1.0,
                alpha=0.6, zorder=1.5)
 
    for d in D:
        if d not in coords:
            continue
        x, y = coords[d]
        ax.scatter(x, y, s=90, color="orange", edgecolor="black",
                   marker="s", zorder=3)
        ax.annotate(str(d), (x, y), textcoords="offset points",
                    xytext=(6, 6), fontsize=9)
 
    for d in depot_ids:
        if d not in coords:
            continue
        x, y = coords[d]
        ax.scatter(x, y, s=140, color="red", edgecolor="black",
                   marker="^", zorder=4)
        ax.annotate(f"depósito {d}", (x, y), textcoords="offset points",
                    xytext=(6, 6), fontsize=9, fontweight="bold")
 
    legend_elems = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor="lightgray",
               markersize=6, label="Vértice"),
        Line2D([0], [0], color="tab:blue", linewidth=2.2,
               label="Segmento a inspecionar"),
        Line2D([0], [0], color="black", linestyle="--", linewidth=1.2,
               label="Arco terrestre (Ap)"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="orange",
               markeredgecolor="black", markersize=9, label="Ponto de paragem (D)"),
        Line2D([0], [0], marker="^", color="w", markerfacecolor="red",
               markeredgecolor="black", markersize=10, label="Depósito"),
    ]
    ax.legend(handles=legend_elems, loc="upper left", fontsize=8)
 
    ax.set_title(data["instance"]["name"])
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    fig.tight_layout()
 
    if out_path:
        fig.savefig(out_path, dpi=150)
        print(f"Figura guardada em: {out_path}")
    else:
        plt.show()
 
 
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python plot_instance.py instancia.json [saida.png]")
        sys.exit(1)
 
    json_path = sys.argv[1]
    out_png = sys.argv[2] if len(sys.argv) > 2 else None
 
    instance = load_instance(json_path)
    plot_instance(instance, out_png)
 
