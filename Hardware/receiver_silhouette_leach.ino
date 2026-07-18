  /*{1, 21.37, 24.11, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {2, 7.62, 65.98, 4.20, false, false,  0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {3, 74.63, 89.28, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {4, 70.73, 67.54, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {5, 53.65, 42.44, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {6, 95.90, 18.32, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {7, 94.48, 3.31, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {8, 28.01, 95.86, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {9, 51.58, 67.39, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {10, 2.20, 22.54, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {11, 26.23, 67.35, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
  {12, 99.24, 82.93, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, 0, false, 0, 0, 0.0, 0.0},
};
*/
// Sink Lora dengan identitas pengirim '999' di setiap pesan
// --- PHASE 1 & 2 SINK SIDE ---
#include <SPI.h>
#include <LoRa.h>

#define SS_PIN 22
#define RST_PIN 15
#define DIO0_PIN 21


const int sinkID = 999;
const int totalNodes = 16;
const unsigned long retryInterval1 = 500;
const unsigned long retryInterval2 = 400;


unsigned long phaseStartTime = 0;
const int maxRetries = 3;

const unsigned long CM_to_CH_duration = 2000;  // Waktu maksimum untuk CM kirim data ke CH
const unsigned long CH_to_Sink_duration = 1400; // Per CH: kirim agregat + tunggu ACK
int currentClusterIndex = 0;
bool waitingForClusterData = false;
unsigned long clusterAggStartTime = 0;
const unsigned long clusterAggTimeout = 8000;

int clstr_CH_list[totalNodes];  // Daftar indeks node yang jadi CH
int clstr_CH_count = 0;
bool aggRequested = false;
float E_residu=2.2 *4.2*3600;

unsigned long timestampREQ1_sent[totalNodes];
unsigned long timestampSENDDATA_sent[totalNodes];
unsigned long timestampAGG_received[totalNodes];
unsigned long timestamp_clstr_sendRole_start = 0;
unsigned long timestamp_clstr_sendRole_end = 0;

int currentRound = 1;
unsigned long T_total = 0;
bool initAlreadySent = false;
unsigned long clusteringStartTime = 0;
enum PhaseState { INIT, CLUSTERING, AGGREGASI };
PhaseState currentPhase = INIT;
bool clusteringInProgress = false;
unsigned long startRoundTime = 0;

struct NodeData {
  int id;
  float posX = 0;
  float posY = 0;
  float voltage = 0;
  float current = 0;
  float arus_awal=0;
  float initialEnergy = 0;
  bool isActive = false;
  bool ack1 = false;
  bool ack_clust = false;
  bool isCH = false;
  int assignedCH = -1; 
  bool ch_ready = false;
  int clusterID = -1;             // Cluster ke berapa
  String aggData = "";            // Data agregasi diterima dari node
  bool hasSentData = false;       // Status apakah sudah mengirim data ke sink
};

NodeData nodes[totalNodes];
// Penerimaan data dari node

struct SensorData {
  int nodeID;
  float voltage;
  float current;
  float temperature;
  float humidity;
  float gas;
};

/*
struct AggData {
  int nodeID;
  float voltage;
  float current;
  float temperature;
  float humidity;
  float gas;
};
*/
SensorData clstr_data[16];  // Misal maksimal 16 node per ronde
int clstr_data_index = 0;

std::vector<SensorData> clusterData;

  static int currentCluster = 1;
  static int currentMemberIndex = 0;
  static bool waitingForResponse = false;
  static unsigned long lastRequestTime = 0;
  static int chID = -1;


const unsigned long initTime = 300;
const unsigned long dataTime = 400;
const unsigned long totalSlotTime = 41600;
const unsigned long idleTime = totalSlotTime - initTime - dataTime;
// Struktur tambahan untuk cluster
float clstr_energy[totalNodes];           // Energi residu tiap node
bool clstr_isCH[totalNodes];              // Status apakah node adalah CH
int clstr_cluster_assignments[totalNodes];// CH yang dituju oleh node
int clstr_K = 0;                           // Jumlah cluster terbentuk
bool aggregationInProgress = false;
void startClusterAggregation();//int clusterIndex);
bool isClusterDataComplete(int clusterIndex);
void printClusterData(int clusterIndex);
void receiveAggregatedData();

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(1000);

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  Serial.println("Sink initialized. Starting Round 1");
  for (int i = 0; i < totalNodes; i++) nodes[i].id = i + 1;
  LoRa.setTxPower(8);
  startRoundTime = millis();
  //phase1_requestAllNodes();
}

// ===================================
// SYSTEM CONTROL (LOOP & TRANSITION)
// ===================================
void loop() {
  switch (currentPhase) {
    case INIT:
      if (!initAlreadySent) {
        phase1_requestAllNodes();   // ✅ hanya dikirim sekali
        initAlreadySent = true;     //  kunci agar tidak dikirim lagi
      }

      if (millis() - startRoundTime >= totalNodes * 1000) {
        endInitPhase();             //  pindah ke fase berikut
        //initAlreadySent = false;    // 🔓 reset flag untuk ronde berikutnya
      }
      break;

    case CLUSTERING:
      Serial.println("📌 Masuk ke fase CLUSTERING");
      if (!clusteringInProgress) {
          if (currentRound % 1 == 0) { runClusteringPhase(); }
          if (allCHAcknowledged()) {
            currentPhase = AGGREGASI;
            startRoundTime = millis();
          } else if (millis() - clusteringStartTime > 5000) {
            Serial.println("⚠️ Timeout ACK_cluster. Lanjut ke AGGREGASI secara paksa.");
            currentPhase = AGGREGASI;
            startRoundTime = millis();
          } else {
            Serial.println("⏳ Menunggu semua CH mengirim ACK_cluster...");
          }
        }
        break;

    case AGGREGASI:
      if (!aggregationInProgress) {
        currentClusterIndex = 0;
        aggregationInProgress = true;
        //Serial.println("🔁 Cluster " + String(currentClusterIndex - 1) + " selesai. Mulai cluster " + String(currentClusterIndex));
        startClusterAggregation();//currentClusterIndex);  // Mulai dari cluster pertama
      } else {
        //Serial.println("⌛ Menunggu AGGDATA selama 5 detik...");
        unsigned long t_wait = millis();
        while (millis() - t_wait < 10000) {
          receiveAggregatedData();   // Menerima dan ACK data dari node aktif
        }

        // Cek apakah semua anggota cluster saat ini sudah mengirimkan data
        if (isClusterDataComplete(currentClusterIndex)) {
          printClusterData(currentClusterIndex);  // Cetak tabel cluster
          currentClusterIndex++;  // Lanjut ke cluster berikutnya

          if (currentClusterIndex < clstr_K) {
            startClusterAggregation();//currentClusterIndex);  // Mulai pengumpulan untuk cluster selanjutnya
          } else {
            // Semua cluster selesai → reset dan lanjut ronde berikutnya
            Serial.println("✅ Semua data cluster telah diterima dan dicetak.");
            aggregationInProgress = false;
            resetNodes();            // reset data internal Sink
            currentPhase = INIT;     // kembali ke fase INIT
            startRoundTime = millis();
          }
        }
      }
      break;
  }
}



/*
void loop() {
  if (clusteringInProgress || millis() - startRoundTime < 20000) {
    // Jangan lakukan apa-apa
    return;
  }
  if (millis() - startRoundTime >= totalNodes * 1000) {
    runInitPhaseAndNextRound();
  }
  receiveResponses();
}
*/
void endInitPhase() {
  printPhase1Table();  // Menampilkan hasil INIT dari ronde saat ini
  currentPhase = CLUSTERING;
  startRoundTime = millis();
}
/*
void runInitPhaseAndNextRound() {
  printPhase1Table();
  clstr_announceNewRound(); //==== pengumuman akhir rou
  currentRound++;
  startRoundTime = millis();
  resetNodes();
  phase1_requestAllNodes();
}
*/
void clstr_announceNewRound() {
  LoRa.beginPacket();
  LoRa.print(String(sinkID) + ";NEWROUND;");
  LoRa.endPacket();
  Serial.println("📢 [Sink] Mengirim sinyal NEWROUND ke semua node.");
}
//======================================= Initial Process ========================
// =============================
// 1. REQUEST PHASE (REQ1)
// =============================

void phase1_requestAllNodes() {
  for (int i = 0; i < totalNodes; i++) {
    requestSingleNode(i);
  }
}

void requestSingleNode(int index) {
  int retry = 0;
  bool received = false;
  while (retry < maxRetries && !received) {
    sendREQ1(nodes[index].id);
    timestampREQ1_sent[index] = millis();  // ⏱️ Timestamp REQ1 sent
    unsigned long startWait = millis();
    while (millis() - startWait < retryInterval1) {
      receiveResponses();
      if (nodes[index].ack1) {
        received = true;
        break;
      }
    }
    retry++;
  }
  if (!nodes[index].ack1) {
    Serial.println("Node " + String(nodes[index].id) + " did not respond to REQ1.");
    nodes[index].isActive = false;
  }
}

void sendREQ1(int nodeId) {
  String message = String(sinkID) + ";REQ1;" + String(nodeId);
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}

// =============================
// 2. RECEIVE & PROCESS INIT1
// =============================

void receiveResponses() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    if (!msg.startsWith(String(sinkID) + ";")) return;

    msg = msg.substring(msg.indexOf(';') + 1);
    if (msg.startsWith("INIT1")) {
      processINIT1Message(msg);
    }
  }
}

void processINIT1Message(String msg) {
  int nodeId = msg.substring(6, msg.indexOf(';', 6)).toInt();
  String data = msg.substring(msg.indexOf(';', 6) + 1);

  int sepXY = data.indexOf(';');
  String xyPart = data.substring(0, sepXY);
  String vcPart = data.substring(sepXY + 1);

  int commaXY = xyPart.indexOf(',');
  float x = xyPart.substring(0, commaXY).toFloat();
  float y = xyPart.substring(commaXY + 1).toFloat();

  int commaVC = vcPart.indexOf(',');
  float voltage = vcPart.substring(0, commaVC).toFloat();
  float current = vcPart.substring(commaVC + 1).toFloat();

  updateNodeData(nodeId, x, y, voltage, current);
  timestampSENDDATA_sent[nodeId - 1] = millis();  // ⏱️ Timestamp INIT1 received
  sendACK1(nodeId);
}

void updateNodeData(int nodeId, float x, float y, float voltage, float current) {
  nodes[nodeId - 1].posX = x;
  nodes[nodeId - 1].posY = y;
  nodes[nodeId - 1].voltage = voltage;
  nodes[nodeId - 1].current = current;
  nodes[nodeId - 1].initialEnergy = 4.2;
  nodes[nodeId - 1].ack1 = true;
  nodes[nodeId - 1].isActive = true;
}

// =============================
// 3. ACK PHASE (ACK1)
// =============================

void sendACK1(int nodeId) {
  String message = String(sinkID) + ";ACK1;" + String(nodeId);
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  timestampAGG_received[nodeId - 1] = millis();  // ⏱️ Timestamp ACK1 sent
  Serial.println("ACK1 sent to Node " + String(nodeId));
}

// =============================
// 4. DISPLAY RESULT (TABEL INIT)
// =============================

void printPhase1Table() {
  Serial.println("\n=== PHASE 1 TABLE (Round " + String(currentRound) + ") ===");
  Serial.println("| ID |   X   |   Y   |Voltage(Volt)| Current(mA) | INIT(ms) |");
  Serial.println("============================================================");
  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].isActive) {
      Serial.println("| " + String(nodes[i].id) + "  | " + 
                     String(nodes[i].posX, 2) + " | " + 
                     String(nodes[i].posY, 2) + " |     " + 
                     String(nodes[i].voltage, 2) + "       | " + 
                     String(nodes[i].current, 2) + "   |   " +
                     String(initTime) + "    |");
    }
  }
  Serial.println("============================================================\n");
}

void resetNodes() {
  for (int i = 0; i < totalNodes; i++) {
    nodes[i].ack1 = false;
    nodes[i].isActive = false;
    nodes[i].isCH = false;            // Reset peran Cluster Head
    nodes[i].assignedCH = -1;         // Reset CM ke CH assignment
    nodes[i].voltage = 0;             // (opsional) reset tegangan
    nodes[i].current = 0;             // (opsional) reset arus
  }
  currentMemberIndex = 0;  // jika digunakan
  startRoundTime = millis();
  currentCluster = 1;
  chID = -1;
  waitingForResponse = false;
  lastRequestTime = 0;
  clstr_announceNewRound(); //==== pengumuman akhir rou
  initAlreadySent = false;
  currentRound++;
  clstr_data_index = 0;
  
  //for (int i = 0; i < 10; i++) clusterData[i].clear();
}

// =============================================== Clustering Process ================================================================
// Perhitungan Energi
// Panel: Clustering - CH & CM Selection (Revisi dengan Node Aktif, ACK CH Broadcast sebagai ACK_cluster, dan Prosedur Energi)
// ===================================================================================================================================
// ======================= memanggil 5 tahap ======================
void runClusteringPhase() {
  clusteringInProgress = true;
  clstr_calculateResidualEnergy();
  clstr_determineClusterCount(4);
  clstr_selectCH(0.8, 0.2);
  clstr_sendRoleToEachNode();
  clstr_printClusterAssignments();
  clusteringInProgress = false;
  clusteringStartTime = millis();  
}
// =============================
// CLUSTERING PHASE (5 TAHAPAN)
// =============================

// Timestamp variables (deklarasi global)
unsigned long timestamp_clstr_energy;
unsigned long timestamp_clstr_K;
unsigned long timestamp_clstr_CH_CM;
unsigned long timestamp_clstr_role_sent;
unsigned long timestamp_clstr_table;

// -------------------------------------------------
// 1. Hitung Energi Residu Tiap Node
// -------------------------------------------------
void clstr_calculateResidualEnergy() {
  const float V_supply = 3.7; // Volt
  const float t_tx = 1.0;     // detik
  const float t_idle = 25.0;  // detik
  const float current_idle = 15.0; // mA

  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].isActive) {
      float current_tx = nodes[i].current; // dalam mA
      float E_tx = (V_supply * current_tx * t_tx) / 3600.0;     // mWh
      float E_idle = (V_supply * current_idle * t_idle) / 3600.0; // mWh
      float E_total = E_tx + E_idle;

      clstr_energy[i] = E_residu-T_total;//nodes[i].voltage * nodes[i].current * 120 * 60 - E_total; // energi awal - konsumsi
      E_residu=clstr_energy[i];
    } else {
      clstr_energy[i] = 0;
    }
  }
  timestamp_clstr_energy = millis();
}

int clstr_alive() {
  int count = 0;
  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].isActive) count++;
  }
  return count;
}

// -------------------------------------------------
// 2. Optimasi Jumlah Cluster dengan Silhouette
// -------------------------------------------------
void clstr_determineClusterCount(int klaster) {
  int clstr_alive_count = clstr_alive();
  if (clstr_alive_count == 0) {
    clstr_K = 0;
    Serial.println("Tidak ada node aktif. Jumlah cluster = 0.");
    return;
  }

  if (clstr_alive_count == 1) {
    clstr_K = 1;
    Serial.println("🔹 Hanya 1 node aktif → Jumlah cluster: 1");
    timestamp_clstr_K = millis();
    return;
  }

  int k_min = 2;
  int k_max = min(4, clstr_alive_count);  // batasi max cluster ke 4 atau jumlah node aktif
  float best_silhouette = -1.0;
  int best_k = 1;

  for (int k = k_min; k <= k_max; k++) {
    // Reset cluster state
    for (int i = 0; i < totalNodes; i++) {
      nodes[i].isCH = false;
      nodes[i].assignedCH = -1;
    }

    clstr_K = k;
    clstr_selectCH(0.5, 0.5);  // lakukan seleksi CH untuk nilai k ini

    // Hitung rata-rata silhouette
    float total_silhouette = 0.0;
    int count = 0;

    for (int i = 0; i < totalNodes; i++) {
      if (!nodes[i].isActive) continue;

      int cluster_i = nodes[i].assignedCH;
      float xi = nodes[i].posX;
      float yi = nodes[i].posY;

      float a_i = 0.0;
      float b_i = 99999.0;
      int a_count = 0;

      // Hitung a(i): rata-rata jarak ke node dalam klaster yang sama
      for (int j = 0; j < totalNodes; j++) {
        if (i == j || !nodes[j].isActive) continue;
        if (nodes[j].assignedCH == cluster_i) {
          float dx = xi - nodes[j].posX;
          float dy = yi - nodes[j].posY;
          a_i += sqrt(dx * dx + dy * dy);
          a_count++;
        }
      }
      if (a_count > 0) a_i /= a_count;

      // Hitung b(i): minimum rata-rata jarak ke klaster lain
      for (int c = 0; c < clstr_K; c++) {
        int ch_id = clstr_CH_list[c];
        if (ch_id == cluster_i) continue;
        float sum_dist = 0.0;
        int count_cluster = 0;

        for (int j = 0; j < totalNodes; j++) {
          if (!nodes[j].isActive || nodes[j].assignedCH != ch_id) continue;
          float dx = xi - nodes[j].posX;
          float dy = yi - nodes[j].posY;
          sum_dist += sqrt(dx * dx + dy * dy);
          count_cluster++;
        }

        if (count_cluster > 0) {
          float avg_dist = sum_dist / count_cluster;
          if (avg_dist < b_i) b_i = avg_dist;
        }
      }

      float s_i = 0.0;
      if (a_i < b_i && b_i > 0) {
        s_i = (b_i - a_i) / b_i;
      } else if (a_i > b_i && a_i > 0) {
        s_i = (b_i - a_i) / a_i;
      }

      total_silhouette += s_i;
      count++;
    }

    float avg_silhouette = (count > 0) ? total_silhouette / count : 0.0;

    if (avg_silhouette > best_silhouette) {
      best_silhouette = avg_silhouette;
      best_k = k;
    }
  }

  clstr_K = best_k;
  Serial.print("🔍 Silhouette optimal → Jumlah cluster (K): ");
  Serial.println(clstr_K);
  timestamp_clstr_K = millis();
}


// -------------------------------------------------
// 3. Pemilihan CH dan Penetapan CM
// -------------------------------------------------
void clstr_selectCH(float alpha, float beta) {
  float E_max = 0, E_min = 99999;
  float d_max = 0, d_min = 99999;
  float d_sink[totalNodes];

  int alive_count = clstr_alive();

  // Jika hanya 1 node aktif, langsung jadikan dia CH
  if (alive_count == 1) {
    for (int i = 0; i < totalNodes; i++) {
      nodes[i].isCH = false;
      if (nodes[i].isActive) {
        nodes[i].isCH = true;
        nodes[i].assignedCH = nodes[i].id;
        Serial.println("🔹 Hanya 1 node aktif → langsung jadi CH: Node ID " + String(nodes[i].id));
      }
    }
    clstr_assignCM();
    clstr_assignClusterID();
    clstr_populateCHList();
    timestamp_clstr_CH_CM = millis();
    return;
  }

  // Jika lebih dari 1 node aktif, lanjut proses normal
  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isActive) continue;
    float E = clstr_energy[i];
    if (E > E_max) E_max = E;
    if (E < E_min) E_min = E;

    float dx = nodes[i].posX - 0;
    float dy = nodes[i].posY - 0;
    d_sink[i] = sqrt(dx * dx + dy * dy);
    if (d_sink[i] > d_max) d_max = d_sink[i];
    if (d_sink[i] < d_min) d_min = d_sink[i];
  }

  for (int i = 0; i < totalNodes; i++) {
    nodes[i].isCH = false;
    if (!nodes[i].isActive) continue;

    float E = clstr_energy[i];
    float d = d_sink[i];
    float Pbase = (float)clstr_K / (alive_count + 1e-6);
    float normE = (E_max - E_min > 1e-6) ? (E - E_min) / (E_max - E_min) : 0;
    float normD = (d_max - d_min > 1e-6) ? (d_max - d) / (d_max - d_min) : 0;

    float Pn = Pbase * (alpha * normE + beta * normD);
    int denom = (int)(1.0 / (Pn + 1e-6));
    if (denom <= 0) denom = 1;
    int rmod = currentRound % denom;
    float Tn = Pn / (1 - Pn * (rmod + 1) + 1e-6);
    float randVal = random(0, 1000) / 1000.0;

    if (randVal < Tn) {
      nodes[i].isCH = true;
      nodes[i].assignedCH = nodes[i].id;
    }

  }

  int totalCH = 0;
  for (int i = 0; i < totalNodes; i++) if (nodes[i].isCH) totalCH++;
  if (totalCH == 0) {
    Serial.println("⚠️ Tidak ada CH terpilih, pilih acak.");
    int count = 0;
    while (count < clstr_K) {
      int idx = random(0, totalNodes);
      if (nodes[idx].isActive && !nodes[idx].isCH) {
        nodes[idx].isCH = true;
        count++;
      }
    }
    totalCH = 0;
    for (int i = 0; i < totalNodes; i++) if (nodes[i].isCH) totalCH++;
  }
  clstr_K = totalCH;
  Serial.println("Jumlah Cluster (K) adaptif: " + String(clstr_K));
  clstr_assignCM();
  clstr_assignClusterID();
  clstr_populateCHList();
  timestamp_clstr_CH_CM = millis();
}

void clstr_populateCHList() {
  int idx = 0;
  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].isCH) {
      clstr_CH_list[idx++] = nodes[i].id;
    }
  }
}


void clstr_assignCM() {
  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isCH) {
      nodes[i].assignedCH = -1;  // Hanya reset CM
    }
  }
  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isActive || nodes[i].isCH) continue;

    float minDist = 1e9;
    int nearestCH = -1;

    for (int j = 0; j < totalNodes; j++) {
      if (!nodes[j].isActive || !nodes[j].isCH) continue;

      float dx = nodes[i].posX - nodes[j].posX;
      float dy = nodes[i].posY - nodes[j].posY;
      float dist = sqrt(dx * dx + dy * dy);

      if (dist < minDist) {
        minDist = dist;
        nearestCH = nodes[j].id;
      }
    }

    if (nearestCH != -1) {
      nodes[i].assignedCH = nearestCH;  // ✅ SIMPAN ID CH yang ditugaskan ke CM
      Serial.println("Node " + String(nodes[i].id) + " assigned to CH " + String(nearestCH));
    } else {
      Serial.println("⚠️ Node " + String(nodes[i].id) + " tidak mendapat CH.");
    }
  }
}
void clstr_assignClusterID() {
  int clusterIdx = 1;
  // Map dari CH.id → cluster index
  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isActive || !nodes[i].isCH) continue;

    int chID = nodes[i].id;
    nodes[i].clusterID = clusterIdx;

    // Beri clusterID yang sama ke semua CM yang assignedCH = chID
    for (int j = 0; j < totalNodes; j++) {
      if (!nodes[j].isActive || nodes[j].isCH) continue;
      if (nodes[j].assignedCH == chID) {
        nodes[j].clusterID = clusterIdx;
      }
    }

    clusterIdx++;
  }
}


// -------------------------------------------------
// 4. Penyampaian Informasi ke Node + Permintaan ACK
// -------------------------------------------------
void clstr_sendRoleToEachNode() {
  Serial.println("\n⏩ [CLSTR] Kirim Role CH/CM ke semua node aktif...");
  timestamp_clstr_sendRole_start = millis();

  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isActive) continue;

    int attempt = 0;
    bool acknowledged = false;
    unsigned long startTime = millis();

    while (millis() - startTime < 1000 && attempt < 3) {  // ⏱ Retry maksimal 2x, timeout 500 ms
      String role;
      if (nodes[i].isCH) {
        role = "CH";
      } else if (nodes[i].assignedCH == -1) {
        role = "ISOLATED";  // atau "None" 
      } else {
        role = "CM";
      }

      String msg = String(sinkID) + ";CLINFO;" + nodes[i].id + ";" + role + ";" + nodes[i].assignedCH;

      LoRa.beginPacket();
      LoRa.print(msg);
      LoRa.endPacket();
      Serial.println("📤 Kirim ke Node " + String(nodes[i].id) + ": " + msg);

      unsigned long ackStart = millis();
      while (millis() - ackStart < 1000) {  // tunggu ACK maksimal 300ms
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
          String incoming = "";
          while (LoRa.available()) {
            incoming += (char)LoRa.read();
          }
          if (incoming.startsWith("999;ACK_cluster;")) {
            int ackNode = incoming.substring(16).toInt();
            if (ackNode == nodes[i].id) {
              Serial.println("✅ ACK dari Node " + String(ackNode));
              nodes[i].ack_clust = true;
              acknowledged = true;
              break;
            }
          }
        }
      }
      if (acknowledged) break;
      attempt++;
    }
    if (!acknowledged) {
      Serial.println("⚠️ Node " + String(nodes[i].id) + " tidak membalas ACK_cluster.");
    }
    delay(75);  // ⏳ Delay antar node dipangkas untuk efisiensi namun tetap aman
  }

  timestamp_clstr_sendRole_end = millis();
}


void receiveACK_clus() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    if (msg.startsWith(String(sinkID) + ";ACK_cluster;")) {
      int senderID = msg.substring(msg.lastIndexOf(';') + 1).toInt();
      for (int i = 0; i < totalNodes; i++) {
        if (nodes[i].id == senderID) {
          nodes[i].ack_clust = true;
          Serial.println("✅ ACK_cluster diterima dari Node ID: " + String(senderID));
          break;
        }
      }
    }
  }
}

bool allCHAcknowledged() {
  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].isActive && nodes[i].isCH && !nodes[i].ack_clust) {
      return false;  // Ada CH yang belum ACK
    }
  }
  return true;
}

// -------------------------------------------------
// 5. Visualisasi dalam Bentuk Tabel
// -------------------------------------------------
void clstr_printClusterAssignments() {
  Serial.println("\n=== TABEL CLUSTER ASSIGNMENT ===");
  Serial.println("| Node ID |   Peran   | CH ID | Cluster No |");
  Serial.println("============================================");

  int clusterNumbers[totalNodes];
  for (int i = 0; i < totalNodes; i++) clusterNumbers[i] = -1;
  int clusterIndex = 1;
  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].isCH) clusterNumbers[i] = clusterIndex++;
  }

  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isActive) continue;
    String peran = nodes[i].isCH ? "CH" : "CM";
    int chID = nodes[i].isCH ? nodes[i].id : nodes[i].assignedCH;
    int chIdx = -1;
    for (int j = 0; j < totalNodes; j++) {
      if (nodes[j].id == chID) {
        chIdx = j;
        break;
      }
    }
    int clusterNo = (chIdx >= 0) ? clusterNumbers[chIdx] : -1;

    Serial.printf("|   %2d    |   %-3s     |  %2d   |     %2d      |\n",
                  nodes[i].id, peran.c_str(), chID, clusterNo);
  }
  Serial.println("============================================\n");
  timestamp_clstr_table = millis();
}

// ============================================ Fase Agregasi Data CM-CH-Sink ============================
// ===============Proses Transmisi Data Agregasi oleh CH dari CM ; CH dikirim ke Sink ==================||
//========================================================================================================

void startClusterAggregation() {
  currentClusterIndex = 1;  // mulai dari cluster 1
  waitingForClusterData = false;
  clusterAggStartTime = millis();
  Serial.println("\n=== MULAI FASE AGGREGASI ===");
  requestClusterData(currentClusterIndex);
}

void requestClusterData(int clusterNo) {
  Serial.println("\n🚩 Meminta data dari Cluster " + String(clusterNo));
  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isActive) continue;
    if (nodes[i].assignedCH == clstr_CH_list[clusterNo - 1] || nodes[i].id == clstr_CH_list[clusterNo - 1]) {
      String msg = String(sinkID) + ";REQ_AGG;" + String(nodes[i].id);
      LoRa.beginPacket();
      LoRa.print(msg);
      LoRa.endPacket();
      delay(75); // Hindari tabrakan
      Serial.println("📤 Kirim REQ_AGG ke Node " + String(nodes[i].id) + " → " + msg);
    }
  }
  waitingForClusterData = true;
  clusterAggStartTime = millis();
}

void receiveAggregatedData() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  // Baca pesan dari LoRa
  String msg = "";
  while (LoRa.available()) {
    char c = (char)LoRa.read();
    msg += c;
    //Serial.print(c);  // Monitoring real-time karakter masuk
  }
  Serial.println(); // newline setelah pesan
  //Serial.println("📩 Raw message AGGDATA diterima: " + msg);
  msg.trim();

  // Validasi format awal pesan
  if (!msg.startsWith(String(sinkID) + ";AGGDATA;")) return;

  // Ambil payload setelah "AGGDATA;"
  String payload = msg.substring(msg.indexOf("AGGDATA;") + 8);

  // Pisahkan ID node
  int idEnd = payload.indexOf(";");
  if (idEnd == -1) return;
  int nodeID = payload.substring(0, idEnd).toInt();

  String rest = payload.substring(idEnd + 1);
  int splitIndex = rest.indexOf(";");
  if (splitIndex == -1) return;

  String voltCurr = rest.substring(0, splitIndex);        // 3.28,707.22
  String sensorData = rest.substring(splitIndex + 1);     // 32.80,58.00,2.35

  // Parsing voltase dan arus
  float voltage = voltCurr.substring(0, voltCurr.indexOf(",")).toFloat();
  float current = voltCurr.substring(voltCurr.indexOf(",") + 1).toFloat();

  // Parsing data sensor: suhu, kelembapan, gas
  float temperature = sensorData.substring(0, sensorData.indexOf(",")).toFloat();
  String rem = sensorData.substring(sensorData.indexOf(",") + 1);
  float humidity = rem.substring(0, rem.indexOf(",")).toFloat();
  float gas = rem.substring(rem.indexOf(",") + 1).toFloat();

  // Simpan ke array klaster
  clstr_data[clstr_data_index++] = {nodeID, voltage, current, temperature, humidity, gas};

  // Tandai node sudah kirim data
  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].id == nodeID) {
      nodes[i].hasSentData = true;
      break;
    }
  }

  // Log lengkap
 /*
  Serial.println("📩 AGGDATA diterima di Sink:");
  Serial.println("🔍 ID: " + String(nodeID));
  Serial.println("  V: " + String(voltage, 2) + ", C: " + String(current, 2));
  Serial.println("  T: " + String(temperature, 2) + ", H: " + String(humidity, 2) + ", G: " + String(gas, 2));
*/

  // Kirim ACK_AGG
  if (nodeID > 0) {
    String ack = String(sinkID) + ";ACK_AGG;" + String(nodeID);
    LoRa.beginPacket();
    LoRa.print(ack);
    LoRa.endPacket();
    Serial.println("✅ Kirim ACK_AGG ke Node " + String(nodeID));
  } else {
    Serial.println("⚠️ ID tidak valid. ACK_AGG tidak dikirim.");
  }
}

void printClusterData(int clusterIndex) {
  Serial.println("=========================================================");
  Serial.println("Node ID | Tegangan | Arus | Temperatur | Kelembaban | Gas");
  Serial.println("---------------------------------------------------------");
  for (int i = 0; i < clstr_data_index; i++) {
    SensorData d = clstr_data[i];
    Serial.print(d.nodeID); Serial.print(" | ");
    Serial.print(d.voltage, 2); Serial.print(" | ");
    Serial.print(d.current, 2); Serial.print(" | ");
    Serial.print(d.temperature, 2); Serial.print(" | ");
    Serial.print(d.humidity, 2); Serial.print(" | ");
    Serial.println(d.gas, 2);
  }
  Serial.println("=========================================================");
}


int getCHID(int clusterIndex) {
  for (int i = 0; i < totalNodes; i++) {
    if (nodes[i].isActive && nodes[i].clusterID == clusterIndex && nodes[i].isCH) {
      return nodes[i].id;
    }
  }
  return -1;
}
bool isClusterDataComplete(int clusterIndex) {
  int chID = clstr_CH_list[clusterIndex];

  bool complete = true;
  for (int i = 0; i < totalNodes; i++) {
    if (!nodes[i].isActive) continue;

    if (nodes[i].assignedCH == chID || nodes[i].id == chID) {
      Serial.print("🔍 Cek Node ");
      Serial.print(nodes[i].id);
      Serial.print(" → hasSentData = ");
      Serial.println(nodes[i].hasSentData);

      if (!nodes[i].hasSentData) {
        complete = false;
      }
    }
  }
  return complete;
}
