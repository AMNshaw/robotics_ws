# LIO-SLAM-Shaw 技術總結 / Postmortem

**日期**：2026-04-17  
**系統**：ROS 2 Humble · C++17 · GTSAM ISAM2 · ikd-Tree · Point-to-Plane ICP · Loose Coupling

---

## 零、完整踩坑時間軸（從跑 bag 開始）

### 坑 1 — `CMAKE_BUILD_TYPE` 沒設，跑 Debug build
**症狀**：Scan matching 每幀 ~500ms，系統完全跟不上 10Hz LiDAR，queue 爆炸。  
**誤判**：以為是演算法太慢或 ikd-Tree 有問題。  
**真因**：colcon 預設不帶優化旗標，`-O0` 下 Eigen + PCL 慢了約 50–100 倍。  
**修法**：`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`，每幀降到 ~1ms。  
**教訓**：效能問題先確認編譯設定，Eigen/PCL 對優化極度敏感。

---

### 坑 2 — GTSAM `IndeterminantLinearSystemException` 崩潰
**症狀**：第 2–5 幀必崩，拋出 `IndeterminantLinearSystemException`。  
**誤判**：以為是 factor graph 建構邏輯有 bug。  
**真因**：前幾幀沒有足夠約束就送進 ISAM2；H 矩陣奇異（rank deficient）。  
**修法**：加入狀態機（`WAITING_FOR_FIRST_FRAME → OPTIMIZING`），確保首幀有完整的 PriorFactor(pose + vel + bias) 才開始優化。改用 ISAM2 QR factorization 取代 Cholesky（數值更穩）。  
**教訓**：圖優化的初始化比優化本身更容易出問題。

---

### 坑 3 — IMU 噪聲參數錯了 25–156 倍
**症狀**：120s 表現還行，300s 後軌跡慢慢飄。Bias 估計看起來不穩定。  
**誤判**：以為是 bias init 不夠準。  
**真因**：`imu_acc_noise` 設成 `0.1`（實際應為 `3.99e-3`），`imu_gyr_noise` 設成 `0.1`（實際應為 `1.56e-3`）。過大的噪聲讓 ISAM2 不信任 IMU，gravity alignment 失效。  
**修法**：從 LIO-SAM park dataset 的 `config.yaml` 取得 Allan variance 分析後的正確值。  
**教訓**：IMU noise 參數必須來自感測器標定，不能隨意猜測。

---

### 坑 4 — IMU queue race condition
**症狀**：偶發性 IMU 資料亂序，preintegration 結果跳動。  
**真因**：`getBatchImuData()` 在讀取時對 queue 有側效應（邊讀邊 pop），前端和 IMU callback 同時訪問發生競爭。  
**修法**：改成 read-only snapshot，不在讀取時修改 queue；另外用 timestamp gate 決定何時 pop。  
**教訓**：共享資料結構的讀寫分離是基本功，尤其在多執行緒 ROS callback 環境。

---

### 坑 5 — 靜態 bias 初始化：採樣窗口被行車資料污染
**症狀**：gyr_z 估計值每次重啟都不同（-0.213、-0.326 等），與真值 -0.349 差距大。  
**真因**：`static_imu_buf_` 在重置後繼續累積資料，把行車中的 IMU 測量也算進靜態均值。  
**修法**：加入 `static_imu_done_` flag，只收錄第一個 LiDAR frame 到達之前的靜態資料；加入 `kMinStaticSamples = 200` gate（~0.4s）確保樣本數足夠。  
**教訓**：靜態初始化窗口必須有明確的開始和結束邊界。

---

### 坑 6 — Multi-frame batch 初始化（VINS-Mono 風格）反而更差
**症狀**：換成 10 幀 LM 批次優化後，gyr_z 估計 -0.178（原本靜態均值已有 -0.349）。  
**真因**：批次優化的 solver loop 有個舊的 `init_frames_buf_.size() < 2` guard 沒有被移除，導致直接回傳零 bias。  
**修法**：發現 guard 後移除，但後來確認靜態均值本身就已夠用，直接廢棄 batch init。  
**教訓**：替換核心邏輯後要徹底清除舊的 guard / early return，否則新邏輯永遠執行不到。

---

### 坑 7 — ISAM2 在 20 幀內把 bias 從 -0.41 拉回 0
**症狀**：靜態 init 正確（gyr_z = -0.413），但 ISAM2 跑 20 幀後 bias 變成 -0.26，50 幀後 -0.18。  
**誤判 1**：bias_between_noise 太大 → 調小 → bias 被凍死，不會修正。  
**誤判 2**：bias_prior σ 太大 → 調到 0.01 → bias 短期穩定但仍然在慢慢被拉。  
**真因**：這是可觀測性問題的症狀，不是噪聲參數問題。ISAM2 看不到旋轉誤差，只能把誤差歸因給 bias，自然把 bias 往「讓 pose 說得通」的方向推。  
**教訓**：調參能改變漂移速度，但無法解決根本的不可觀測性。

---

### 坑 8 — Rotation Regularization 的 Catch-22 ← 最終根因確認
**嘗試**：在 ICP Gauss-Newton solver 加入旋轉正規化項，以 IMU 預測旋轉為參考。  
**結果**：  
- σ = 0.1（鬆）→ gyr_z 仍被 ISAM2 拉走，效果等同沒加  
- σ = 0.01（緊）→ ICP 被鎖在 IMU 旋轉上，ISAM2 永遠看不到旋轉誤差，bias 往錯誤方向漂（-0.41 → -0.46）  

**結論**：根本矛盾在於參考本身就帶有 bias 誤差的旋轉。不存在一個正確的 σ。  
**教訓**：Workaround 如果在概念上有矛盾，調任何參數都是徒勞。要解決的是架構問題。

---

## 一、從哪裡出問題

系統架構本身是健全的——模組清楚、每個元件可熱插拔。問題根源在**前端選型的組合**：

```
point-to-plane ICP  +  loose coupling (ISAM2 PriorFactor)  +  IMU bias 未知
```

這三個選擇單獨都可以成立，但組合在一起形成了一個**惡性循環**，無法自我修正。

---

## 二、踩坑時間軸

| 階段 | 症狀 | 當時誤判 | 真因 |
|---|---|---|---|
| 初期 | GTSAM `IndeterminantLinearSystemException` | 程式碼 bug | 前幾幀 H 矩陣奇異，狀態未初始化 |
| 中期 | 120s 沒崩，300s 發散 | 噪聲參數太鬆 | IMU noise 參數錯了 25–156 倍 |
| 後期 | 修正噪聲後仍發散，bias 亂跑 | bias init 不良 | gyr bias 估計誤差 0.06 rad/s |
| 深挖 | 不管怎麼調 bias_prior 都無解 | σ 調太緊/太鬆 | **根因：旋轉不可觀，ISAM2 永遠看不到旋轉誤差** |
| 嘗試修 | Rotation regularization | 以為能補旋轉約束 | Catch-22：參考本身是錯的 IMU 旋轉 |

---

## 三、細節收斂過程

### 第一輪：noise 參數

```
測試結果：bias 從 -0.41 → -0.26（20 幀內）
分析：bias_between_noise 只有 3.5e-5
      但每幀 ImuFactor + PriorFactor 的累積拉力 >> prior
→ 確認：noise 不是問題，是可觀測性問題
```

### 第二輪：靜態 bias 初始化

```
靜態 gyr 均值  = -0.412 rad/s
真值（LIO-SAM）≈ -0.349 rad/s
殘差            =  0.063 rad/s → 每幀姿態誤差 0.36°
```

### 第三輪：定量計算發散速度

每幀姿態誤差：

$$
\Delta\theta_{per\_frame} = 0.063\ \text{rad/s} \times 0.1\ \text{s} = 0.0063\ \text{rad} \approx 0.36°
$$

gravity leak 加速度：

$$
a_{leak} = g \cdot \sin(\Delta\theta) \approx 9.8 \times 0.0063 \approx 0.062\ \text{m/s}^2
$$

到達 failureDetection 閾值（30 m/s）的時間：

$$
t = \frac{30}{0.062} \approx 48\ \text{s}
$$

**實測**：bias 開始漂移後，約 66 幀（6.6 秒）觸發重置。數字完全吻合。

---

## 四、LIO-SAM 的兩種殘差：為什麼能可觀？

### 4.1 兩種殘差定義

LIO-SAM 繼承 LOAM 的特徵分類，對每幀點雲分成兩類：

**Planar feature（面特徵）** → **Point-to-Plane 殘差**

給定查詢點 $\mathbf{p}_i$（body frame），地圖中配對的平面法向量 $\mathbf{n}_i$、平面上一點 $\mathbf{q}_i$：

$$
r^{surf}_i = \mathbf{n}_i^\top \left(\mathbf{R}\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i\right)
$$

**Edge feature（線特徵）** → **Point-to-Line 殘差**

給定查詢點 $\mathbf{p}_i$，地圖中配對線段由兩點 $\mathbf{a},\mathbf{b}$ 定義，單位方向 $\hat{\mathbf{d}} = \frac{\mathbf{b}-\mathbf{a}}{\|\mathbf{b}-\mathbf{a}\|}$，令 $\mathbf{P} = \mathbf{R}\mathbf{p}_i + \mathbf{t}$：

$$
\mathbf{r}^{edge}_i = \left(\mathbf{P} - \mathbf{a}\right) \times \hat{\mathbf{d}}
$$

即查詢點到地圖線的垂直距離向量（3D）。

---

### 4.2 對 Yaw 旋轉的 Jacobian 推導

設旋轉微小擾動 $\delta\boldsymbol{\phi}$（李代數），左擾動模型：

$$
\mathbf{R} \leftarrow \exp(\delta\boldsymbol{\phi})\,\mathbf{R}
\implies
\frac{\partial \mathbf{P}}{\partial \delta\boldsymbol{\phi}} = -\left[\mathbf{R}\mathbf{p}_i\right]_\times
$$

其中 $[\cdot]_\times$ 為反對稱矩陣。對 yaw 分量（$\delta\boldsymbol{\phi} = [0,0,\delta\psi]^\top$）：

$$
\frac{\partial \mathbf{P}}{\partial \delta\psi}
= [0,0,1]^\top \times (\mathbf{R}\mathbf{p}_i)
= \begin{bmatrix}-(\mathbf{R}\mathbf{p}_i)_y \\ (\mathbf{R}\mathbf{p}_i)_x \\ 0\end{bmatrix}
$$

#### Point-to-Plane（水平地面，$\mathbf{n}_i = [0,0,1]^\top$）

$$
\frac{\partial r^{surf}_i}{\partial \delta\psi}
= \mathbf{n}_i^\top \frac{\partial \mathbf{P}}{\partial \delta\psi}
= [0,0,1] \cdot \begin{bmatrix}-(\mathbf{R}\mathbf{p}_i)_y \\ (\mathbf{R}\mathbf{p}_i)_x \\ 0\end{bmatrix}
= \mathbf{0}
$$

**Yaw Jacobian = 0。水平地面的 point-to-plane，yaw 完全不可觀。**

#### Point-to-Line（垂直線，如電線桿、牆角，$\hat{\mathbf{d}} = [0,0,1]^\top$）

$$
\mathbf{r}^{edge}_i = -[\hat{\mathbf{d}}]_\times\,(\mathbf{P} - \mathbf{a})
$$

對 yaw 的 Jacobian：

$$
\frac{\partial \mathbf{r}^{edge}_i}{\partial \delta\psi}
= -[\hat{\mathbf{d}}]_\times \frac{\partial \mathbf{P}}{\partial \delta\psi}
= -\begin{bmatrix}0&-1&0\\1&0&0\\0&0&0\end{bmatrix}
\begin{bmatrix}-(\mathbf{R}\mathbf{p}_i)_y\\(\mathbf{R}\mathbf{p}_i)_x\\0\end{bmatrix}
= \begin{bmatrix}(\mathbf{R}\mathbf{p}_i)_x\\(\mathbf{R}\mathbf{p}_i)_y\\0\end{bmatrix}
$$

只要點不在 z 軸上（$\|\mathbf{R}\mathbf{p}_i\|_{xy} \neq 0$），**Jacobian ≠ 0，yaw 可觀。**

---

### 4.3 兩種殘差的幾何互補性

| 殘差類型 | 約束的 DOF | Yaw 可觀？ | 物理直覺 |
|---|---|---|---|
| Point-to-Plane（水平面） | z、roll、pitch、xy translation | ✗ | 站在地板上，你能感知高低和傾斜，但不知道朝哪轉 |
| Point-to-Plane（垂直面，如牆） | x 或 y translation | 部分 ✓ | 牆的法向量有水平分量，yaw 有微弱約束 |
| **Point-to-Line（垂直線）** | **xy translation + yaw** | **✓** | 繞著一根電線桿轉，距離變化直接反映旋轉 |

**LIO-SAM 使用兩者組合**，在典型室外場景（地面＋垂直結構）下實現 6 DOF 全可觀：

```
地面點雲 (surf) → point-to-plane → 約束 z, roll, pitch
垂直結構 (edge) → point-to-line  → 約束 yaw, xy
```

這就是為什麼 LIO-SAM 能在公園場景穩定運行，而純 point-to-plane（我們的系統）不行——公園有大量水平地面，但垂直線特徵（樹幹、燈柱）被 passthrough extractor 丟掉了。

---

## 五、純 Point-to-Plane 的不可觀性（數學確認）

Point-to-plane 殘差定義：

$$
r_i = \mathbf{n}_i^\top \left( \mathbf{R}\,\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i \right)
$$

對旋轉（繞軸 $\boldsymbol{\phi}$ 的無窮小更新）的 Jacobian：

$$
J_{rot,i} = -\mathbf{n}_i^\top \left[\mathbf{R}\mathbf{p}_i\right]_\times
$$

**在水平地面場景**，所有平面法向量 $\mathbf{n}_i = [0,\ 0,\ 1]^\top$，代入後繞 z 軸旋轉的分量：

$$
J_{yaw,i}
= -[0,0,1]^\top
\begin{bmatrix}
0 & -z_i & y_i \\
z_i & 0 & -x_i \\
-y_i & x_i & 0
\end{bmatrix}
= \mathbf{0}
$$

Hessian 的 yaw-yaw 元素：

$$
H_{yaw,yaw} = \sum_i J_{yaw,i}^2 = 0
$$

矩陣奇異，Gauss-Newton 解不出 yaw 更新量。即使場景有牆和樹，也只是讓這個值從 0 變到很小的數，仍比 edge-to-line 弱 10–100 倍。

**這是物理上的不可觀測性，不是調參能解決的。**

---

## 五點五、Loose Coupling 失效的精確機制

> 以下是對根因的精確描述，澄清一個常見的誤解。

### 誤解：「Loose coupling 沒有傳 covariance」

不對。我們的系統確實有傳 covariance，`correction_noise_` 為：

```
rot:   σ = 0.05 rad（x, y, z 三軸相同）
trans: σ = 0.10 m （x, y, z 三軸相同）
```

### 真正的問題：Covariance 是 Isotropic 的

ISAM2 收到的 PriorFactor 宣稱：

> 「Yaw 已知，σ = 0.05 rad」

但 scan matcher 根本**沒有** yaw 的量測資訊——它吐出的 yaw 只是 IMU 預測值原封不動傳回來。ISAM2 把這個「yaw 量測」當真，發現 IMU 積分的 yaw 和這個「量測 yaw」之間有殘差，選擇把殘差歸咎給 bias（因為 bias 的 BetweenFactor 比較鬆）——**bias 就被拉走了**。

`is_degenerate` flag 確實放寬了 cov，但它是對整個 6 DOF 等比例放寬，不是只放寬 yaw，效果有限。

### iEKF 為什麼不會這樣

iEKF 的 Kalman gain，當 $H_{yaw} \approx 0$：

$$
K_{yaw} = \frac{P_{yaw}\,H_{yaw}}{H_{yaw}\,P\,H_{yaw}^\top + R} \approx \frac{P_{yaw} \cdot 0}{0 + R} = 0
$$

yaw 的更新量**自動為零**，$P_{yaw}$ 不縮小（保持不確定）。系統不會幻覺出一個它根本沒觀測到的 yaw 修正。這是數學的自然結果，不需要手動設定任何旗標。

### 一句話總結

| | iEKF | Loose Coupling（本系統）|
|---|---|---|
| Per-DOF 信息量 | **$H$ 矩陣天然攜帶**，Kalman gain 自動加權 | **需要手動**反映到 prior cov 結構 |
| Yaw 不可觀時 | $K_{yaw} = 0$，自動忽略 | ISAM2 仍信任 yaw 量測，把誤差推給 bias |
| 需要的先驗知識 | 無 | 需要知道「哪個 DOF 是垃圾量測」|

---

## 六、Rotation Regularization 的 Catch-22

嘗試在 ICP solver 加入旋轉正規化項（以 IMU 預測旋轉為參考）：

$$
H \mathrel{+}= \frac{1}{\sigma_{rot}^2} I_{3\times3},\quad
b \mathrel{+}= \frac{1}{\sigma_{rot}^2} \log\left(R_{IMU}^\top R_{curr}\right)
$$

結果存在根本矛盾：

```
σ_rot 太緊 (0.01 rad)            σ_rot 太鬆 (0.1 rad)
────────────────────────         ────────────────────────
ICP 被鎖在 IMU 旋轉上             跟沒加一樣
→ ISAM2 看到「量測 = 預測」       → point-to-plane 旋轉太弱
→ 認為沒有誤差                    → bias 被 ISAM2 拉走
→ bias 永遠不會被修正              → 旋轉發散
        (test25)                          (test24)
```

不存在一個能正確工作的 σ_rot，因為**參考值本身就帶有 bias 誤差的旋轉**。

---

## 七、為什麼 iEKF（FAST-LIO）能做到

### Loose coupling 的信息流（本系統）

```
LiDAR scan match
      ↓
  pose estimate  ──→  ISAM2 PriorFactor(pose)
                              ↑
            ISAM2 只看到「scan match 給的 pose 數字」
            旋轉不準確對 bias 毫無可見性
```

### iEKF（FAST-LIO）的信息流

```
IMU propagation → predicted state x̄  (包含 pose, vel, bias)
                          ↓
LiDAR point residuals → 直接對 (R, t, v, b_g, b_a) 聯合更新
```

Kalman gain 計算：

$$
K = PH^\top\left(HPH^\top + R\right)^{-1}
$$

其中 $P$ 是包含 bias 協方差的**聯合矩陣**，$H$ 包含每個點雲殘差對每個狀態分量的偏導數。

### 關鍵差異

| | Loose Coupling（本系統） | iEKF（FAST-LIO） |
|---|---|---|
| Bias 如何被修正 | 透過 pose 的不一致性間接推斷 | Kalman gain 直接分配到 bias |
| 旋轉誤差如何傳播 | 只影響 pose prior，bias 看不見 | 直接影響聯合狀態向量 |
| 旋轉不可觀時 | bias 完全不受限，自由漂移 | $P_{bb}$ 仍會增長，下一幀更容易修正 |
| 1 幀數據不夠時 | 下一幀 prior 已被污染 | $P$ 矩陣保留不確定性，不貿然更新 |

**一句話**：loose coupling 把 scan match 結果壓縮成一個 pose 數字，**把不確定性資訊丟棄了**。iEKF 保留了每個點雲殘差對每個狀態分量的偏導數，讓數學自己決定哪個狀態能被觀測、應該修正多少。

---

## 八、下一步

### 方案：加入 LOAM 特徵提取

現在系統的瓶頸在前端，只需換掉三個元件，其餘（loop closure、ISAM2、map builder）全部保留：

```
現在
  PassthroughFeatureExtractor
    └── IkdTreeScanMatcher (point-to-plane only)
         └── IkdTreeMapBuilder (單棵 ikd-tree)

目標
  LoamFeatureExtractor  (edge + surf 分類)
    └── LoamScanMatcher  (edge-to-line + point-to-plane)
         └── ILoamMapBuilder  (兩棵 ikd-tree，分別存 edge/surf)
```

### 設計原則

- `ILoamMapBuilder` 繼承現有 `IMapBuilder` 介面，不破壞 loop closure
- Loop closure 仍使用 surf map（`searchKNearestPoints()`）
- 新增 `searchKNearestEdgePoints()` 僅供 `LoamScanMatcher` 使用
- `Keyframe.features.edge` / `.surf` 欄位已存在，不需改 struct

### 為什麼 edge feature 能解決旋轉問題

Edge keypoint 對應的是線段，其 line-to-line 殘差的 Jacobian：

$$
J_{yaw,edge} = \mathbf{d}^\top \left[\mathbf{R}\mathbf{p}_i\right]_\times
$$

其中 $\mathbf{d}$ 是線方向向量。水平場景中牆角、柱子、樹幹的線方向有 $d_x, d_y \neq 0$，yaw Jacobian **不為零**，Hessian 旋轉 block 可逆，旋轉從此可觀。

---

## 九、系統架構現況

```
SlamNode
  └── SlamProcessor
        ├── FrontEnd (LiDAR thread)
        │     ├── ImuDeskewPreprocessor  ✓
        │     ├── PassthroughFeatureExtractor  ← 換成 LoamFeatureExtractor
        │     ├── IkdTreeScanMatcher  ← 換成 LoamScanMatcher
        │     ├── IkdTreeMapBuilder  ← 換成 ILoamMapBuilder
        │     └── GtsamImuPreintegrator  ✓ (不需動)
        └── BackEnd (backend thread)
              ├── GtsamMapOptimizer  ✓
              └── IkdTreeLoopClosureDetector  ✓ (自動受益)
```

---

*模組化架構的價值在此體現：所有踩坑期間寫的 loop closure、map builder、ISAM2 配置一行不浪費，換前端即可。*
