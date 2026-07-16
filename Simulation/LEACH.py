import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm

# === Data node ===
nodes = {
    "Node": ["Sink","Node 1", "Node 2", "Node 3", "Node 4", "Node 5",
             "Node 6", "Node 7", "Node 8", "Node 9", "Node 10", "Node 11", 
             "Node 12", "Node 13", "Node 14", "Node 15", "Node 16"],
    "X":    [30, 30, 47, 30, 23,  9, 54, 36,  5, 48, 13, 22,  8, 14, 21, 43, 40],
    "Y":    [20, 25, 10, 32, 10, 26, 34,  5,  2, 25, 19, 34, 16,  4, 21, 13, 32]
}
df_nodes = pd.DataFrame(nodes)
X_all = df_nodes[['X', 'Y']].values
X = X_all[1:]  # tanpa Sink
SINK = X_all[0]

# === Parameter Energi dan LEACH ===
E_INIT = 1.0
P = 0.2
E_ELEC = 50e-9
E_AMP = 100e-12
PACKET_SIZE = 4000

energy = np.ones(len(X)) * E_INIT
G = np.zeros(len(X))  # epoch counter

# === Clustering hasil silhouette (K=3), diperbaiki label Node 11 ke Cluster 2 ===
best_k = 3
labels = np.array([0,2,1,0,0,1,2,0,1,0,1,0,0,0,2,1])
colors = cm.tab10(np.linspace(0, 1, best_k))

def compute_centroids(data, labels, k):
    return np.array([np.mean(data[labels == i], axis=0) for i in range(k)])

centroids = compute_centroids(X, labels, best_k)

# === Visualisasi Clustering awal ===
plt.figure(figsize=(8,6))
for i in range(best_k):
    cluster_points = X[labels == i]
    for pt in cluster_points:
        plt.plot([pt[0], centroids[i][0]], [pt[1], centroids[i][1]], '--', color=colors[i], linewidth=0.7)
    plt.scatter(cluster_points[:,0], cluster_points[:,1], color=colors[i], label=f"Cluster {i+1}")
    plt.scatter(centroids[i][0], centroids[i][1], marker='P', s=150, color='red', label='Centroid' if i==0 else "")
plt.scatter(SINK[0], SINK[1], marker='X', c='black', s=100, label="Sink")
for i, pos in enumerate(X):
    plt.text(pos[0]+0.5, pos[1], f"Node {i+1}", fontsize=8)
plt.title("Visualisasi Clustering (K=3)")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# === Inisialisasi ronde ke-0 ===
r = 0
alive_per_round = [np.sum(energy > 0)]
energy_per_round = [energy.copy()]
total_energy_consumed = [np.sum(E_INIT - energy)]
CH_record = []

# === Simulasi LEACH ===
while np.any(energy > 0):
    CH_list = []
    clusters = {i: [] for i in range(best_k)}

    for c in range(best_k):
        cluster_indices = np.where(labels == c)[0]
        alive_indices = [i for i in cluster_indices if energy[i] > 0]
        if not alive_indices:
            continue

        eligible = [i for i in alive_indices if G[i] <= 0]
        if not eligible:
            G[alive_indices] = 0
            eligible = alive_indices

        # Pilih CH terdekat ke sink, kemudian berdasarkan energi
        distances = [(i, np.linalg.norm(X[i] - SINK)) for i in eligible]
        distances.sort(key=lambda x: x[1])
        min_dist = distances[0][1]
        nearest_nodes = [i for i, d in distances if np.isclose(d, min_dist, atol=1e-3)]
        if len(nearest_nodes) == 1:
            ch = nearest_nodes[0]
        else:
            ch = max(nearest_nodes, key=lambda i: energy[i])

        CH_list.append(ch)
        G[ch] = int(1 / P)
        for i in alive_indices:
            if i != ch:
                clusters[c].append(i)

    CH_record.append(CH_list)

    for c in clusters:
        ch_list = [ch for ch in CH_list if ch in np.where(labels == c)[0]]
        if not ch_list:
            continue
        ch = ch_list[0]
        for m in clusters[c]:
            d = np.linalg.norm(X[m] - X[ch])
            E_tx = PACKET_SIZE * (E_ELEC + E_AMP * d**2)
            energy[m] -= E_tx
            energy[m] = max(0, energy[m])
        d_sink = np.linalg.norm(X[ch] - SINK)
        E_tx_sink = PACKET_SIZE * (E_ELEC + E_AMP * d_sink**2)
        E_rx = PACKET_SIZE * len(clusters[c]) * E_ELEC
        energy[ch] -= (E_rx + E_tx_sink)
        energy[ch] = max(0, energy[ch])

    r += 1
    alive_per_round.append(np.sum(energy > 0))
    energy_per_round.append(energy.copy())
    total_energy_consumed.append(np.sum(E_INIT - energy))
    G = np.maximum(G - 1, 0)

    if r % 400 == 0 or np.sum(energy > 0) == 0:
        plt.figure(figsize=(7, 7))
        for c in range(best_k):
            cluster_indices = np.where(labels == c)[0]
            alive_nodes = [i for i in cluster_indices if energy[i] > 0]
            dead_nodes = [i for i in cluster_indices if energy[i] == 0]
            for i in alive_nodes:
                plt.scatter(X[i][0], X[i][1], color=colors[c], s=60)
                plt.text(X[i][0]+0.5, X[i][1]+0.5, f"{i+1}", fontsize=8)
            for i in dead_nodes:
                plt.scatter(X[i][0], X[i][1], color='gray', marker='x', s=80)
                plt.text(X[i][0]+0.5, X[i][1]+0.5, f"{i+1}", fontsize=8, color='gray')
        for ch in CH_list:
            if energy[ch] > 0:
                plt.scatter(X[ch][0], X[ch][1], marker='*', c='red', s=200, label="CH")
        plt.scatter(SINK[0], SINK[1], marker='X', c='black', s=100, label="Sink")

        for c in range(best_k):
            ch_list = [ch for ch in CH_list if ch in np.where(labels == c)[0]]
            if not ch_list:
                continue
            ch = ch_list[0]
            for m in clusters[c]:
                if energy[m] > 0:
                    plt.plot([X[m][0], X[ch][0]], [X[m][1], X[ch][1]], 'k--', linewidth=0.5)
            if energy[ch] > 0:
                plt.plot([X[ch][0], SINK[0]], [X[ch][1], SINK[1]], 'r-', linewidth=1.5)

        plt.title(f"Distribusi Node Hidup & Jalur Komunikasi (Ronde {r})")
        plt.legend(loc='upper right', bbox_to_anchor=(1.2, 1))
        plt.grid(True)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.tight_layout()
        plt.show()

plt.figure(figsize=(10, 5))
plt.plot(range(len(alive_per_round)), alive_per_round, marker='o', color='blue')
plt.title("Jumlah Node Hidup per Ronde")
plt.xlabel("Ronde ke-")
plt.ylabel("Jumlah Node Hidup")
plt.grid(True)
plt.tight_layout()
plt.show()

energy_array = np.array(energy_per_round)
plt.figure(figsize=(12, 6))
for i in range(len(X)):
    plt.plot(range(len(energy_array)), energy_array[:, i], label=f"Node {i+1}")
plt.title("Energi Setiap Node per Ronde")
plt.xlabel("Ronde ke-")
plt.ylabel("Energi (Joule)")
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 5))
plt.plot(range(len(total_energy_consumed)), total_energy_consumed, color='green')
plt.title("Total Energi yang Dikonsumsi Jaringan per Ronde")
plt.xlabel("Ronde ke-")
plt.ylabel("Energi Terkonsumsi (Joule)")
plt.grid(True)
plt.tight_layout()
plt.show()

ch_df = pd.DataFrame(CH_record, columns=[f"CH_Cluster_{i+1}" for i in range(best_k)])
ch_df.index.name = "Ronde ke-"
ch_df += 1
ch_df.to_excel("log_cluster_head.xlsx", index=True)
print("Simulasi selesai. Log Cluster Head disimpan ke 'log_cluster_head.xlsx'")
