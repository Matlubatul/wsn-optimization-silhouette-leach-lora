import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import silhouette_score, pairwise_distances
from scipy.spatial.distance import cdist
from matplotlib import cm
import os

# === Data node ===
# Area simulasi: 100 meter x 100 meter
area_width = 100   # sumbu X (meter)
area_height = 100  # sumbu Y (meter)
nodes = {
    "Node": ["Sink", "Node 1", "Node 2", "Node 3", "Node 4", "Node 5",
             "Node 6", "Node 7", "Node 8", "Node 9", "Node 10", "Node 11", 
             "Node 12", "Node 13", "Node 14", "Node 15", "Node 16" ],
    "X":    [30, 30, 47, 30, 23,  9, 54, 36,  5, 48, 13, 22,  8, 14, 21, 43, 40],
    "Y":    [20, 25, 10, 32, 10, 26, 34,  5,  2, 25, 19, 34, 16,  4, 21, 13, 32]
}
df_nodes = pd.DataFrame(nodes)
X_all = df_nodes[['X', 'Y']].values
X = X_all[1:]  # tanpa Sink

def compute_centroids(data, labels, k):
    centroids = []
    for i in range(k):
        pts = data[labels == i]
        if len(pts) > 0:
            centroids.append(np.mean(pts, axis=0))
        else:
            centroids.append(np.zeros(data.shape[1]))
    return np.array(centroids)

def compute_silhouette_components(X, labels):
    n = len(X)
    a_vals, b_vals, s_vals = [], [], []
    distance_matrix = pairwise_distances(X)
    for i in range(n):
        same_cluster = np.where(labels == labels[i])[0]
        if len(same_cluster) > 1:
            a_i = np.mean([distance_matrix[i][j] for j in same_cluster if j != i])
        else:
            a_i = 0

        b_i = np.inf
        for lab in set(labels):
            if lab != labels[i]:
                other = np.where(labels == lab)[0]
                dist = np.mean([distance_matrix[i][j] for j in other])
                if dist < b_i:
                    b_i = dist

        s_i = (b_i - a_i) / max(a_i, b_i) if max(a_i, b_i) > 0 else 0
        a_vals.append(round(a_i, 2))
        b_vals.append(round(b_i, 2))
        s_vals.append(round(s_i, 3))
    return a_vals, b_vals, s_vals

k_range = range(2, 16)
avg_silhouettes = []

output_filename = "hasil_silhouette.xlsx"
output_path = os.path.abspath(output_filename)
print(f"Menyimpan file Excel di: {output_path}")

try:
    with pd.ExcelWriter(output_filename) as writer:
        for k in k_range:
            np.random.seed(42 + k)
            centroids = X[np.random.choice(range(len(X)), k, replace=False)]

            for _ in range(10):
                distances = cdist(X, centroids)
                labels = np.argmin(distances, axis=1)
                new_centroids = compute_centroids(X, labels, k)
                if np.allclose(centroids, new_centroids):
                    break
                centroids = new_centroids

            unique_labels = np.unique(labels)
            if len(unique_labels) < 2:
                print(f"Skip K={k}, cluster aktif kurang dari 2")
                continue

            sil_score = silhouette_score(X, labels)
            avg_silhouettes.append({'K': k, 'Avg_Silhouette': sil_score})

            a_vals, b_vals, s_vals = compute_silhouette_components(X, labels)

            temp_df = df_nodes.copy()
            temp_df['Cluster'] = 0  # Sink = cluster 0
            temp_df.loc[1:, 'Cluster'] = labels + 1  # sensor cluster mulai dari 1
            temp_df['a(i)'] = np.nan
            temp_df['b(i)'] = np.nan
            temp_df['s(i)'] = np.nan
            temp_df.loc[1:, 'a(i)'] = a_vals
            temp_df.loc[1:, 'b(i)'] = b_vals
            temp_df.loc[1:, 's(i)'] = s_vals

            temp_df.to_excel(writer, sheet_name=f"K={k}", index=False)

        df_avg_sil = pd.DataFrame(avg_silhouettes)
        df_avg_sil.to_excel(writer, sheet_name='Rata_Rata_Silhouette', index=False)

    print("✅ File Excel berhasil disimpan.")

except Exception as e:
    print("❌ Gagal menyimpan file Excel:")
    print(e)

# === Plot rata-rata silhouette ===
plt.figure(figsize=(8, 4))
plt.plot(df_avg_sil['K'], df_avg_sil['Avg_Silhouette'], marker='o')
plt.xticks(df_avg_sil['K'])
plt.xlabel("Jumlah Cluster (K)")
plt.ylabel("Rata-rata Silhouette")
plt.title("Evaluasi K-MEANS dengan Silhouette Score")
plt.grid(True)
plt.tight_layout()
plt.show()

# === Visualisasi semua hasil cluster dari K=2 sampai K=14 ===
with pd.ExcelFile(output_filename) as reader:
    for k in k_range:
        try:
            df_k = pd.read_excel(reader, sheet_name=f"K={k}")
            labels = df_k.loc[1:, 'Cluster'].values - 1
            X_k = X_all[1:]  # Tanpa Sink
            centroids = compute_centroids(X_k, labels, k)
            colors = cm.tab20(np.linspace(0, 1, k))

            plt.figure(figsize=(8, 6))
            for i in range(k):
                cluster_points = X_k[labels == i]
                plt.scatter(cluster_points[:, 0], cluster_points[:, 1], label=f'Cluster {i+1}', s=80, color=colors[i])
                for pt in cluster_points:
                    centroid = centroids[i]
                    plt.plot([pt[0], centroid[0]], [pt[1], centroid[1]], color=colors[i], linestyle='--', linewidth=0.7)

            plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=150, marker='P', label='Centroid')
            plt.scatter(X_all[0, 0], X_all[0, 1], c='black', s=120, marker='X', label='Sink')

            for i, txt in enumerate(df_nodes['Node']):
                plt.annotate(txt, (X_all[i, 0]+0.8, X_all[i, 1]+0.8), fontsize=8)

            plt.title(f'Visualisasi Clustering (K={k})')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.show()

        except Exception as e:
            print(f"Gagal memvisualisasikan K={k}: {e}")
            continue

# === Visualisasi Cluster terbaik ===
best_row = df_avg_sil.loc[df_avg_sil['Avg_Silhouette'].idxmax()]
best_k = best_row['K']
print(f"Cluster terbaik (tanpa Sink): K = {best_k}")

with pd.ExcelFile(output_filename) as reader:
    best_df = pd.read_excel(reader, sheet_name=f"K={int(best_k)}")

best_labels = best_df.loc[1:, 'Cluster'].values - 1
best_X = X_all[1:]
best_centroids = compute_centroids(best_X, best_labels, int(best_k))
colors = cm.tab20(np.linspace(0, 1, int(best_k)))

plt.figure(figsize=(8, 6))
for i in range(int(best_k)):
    cluster_points = best_X[best_labels == i]
    plt.scatter(cluster_points[:, 0], cluster_points[:, 1], label=f'Cluster {i+1}', s=80, color=colors[i])
    for pt in cluster_points:
        plt.plot([pt[0], best_centroids[i][0]], [pt[1], best_centroids[i][1]], color=colors[i], linestyle='--', linewidth=0.7)

plt.scatter(best_centroids[:, 0], best_centroids[:, 1], c='red', s=150, marker='P', label='Centroid')
plt.scatter(X_all[0, 0], X_all[0, 1], c='black', s=120, marker='X', label='Sink')

for i, txt in enumerate(df_nodes['Node']):
    plt.annotate(txt, (X_all[i, 0]+0.8, X_all[i, 1]+0.8), fontsize=9)

plt.title(f'Visualisasi Cluster Terbaik (K={int(best_k)})')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# === Gabungkan semua sheet ke Gabungan_Silhouette ===
from openpyxl import load_workbook

gabungan_silhouette = []

for k in k_range:
    try:
        sheet_name = f"K={k}"
        df_k = pd.read_excel(output_filename, sheet_name=sheet_name)
        df_k['K'] = k  # Tambahkan kolom K sebagai penanda
        gabungan_silhouette.append(df_k[df_k['Node'] != 'Sink'])  # hanya node sensor
    except Exception:
        continue

if gabungan_silhouette:
    df_gabungan = pd.concat(gabungan_silhouette, ignore_index=True)
    with pd.ExcelWriter(output_filename, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
        df_gabungan.to_excel(writer, sheet_name="Gabungan_Silhouette", index=False)
    print("✅ Sheet 'Gabungan_Silhouette' berhasil ditambahkan ke file Excel.")
else:
    print("❌ Tidak ada data silhouette yang bisa digabungkan.")
