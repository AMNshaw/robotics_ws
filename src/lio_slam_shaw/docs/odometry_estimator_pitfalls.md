# iEKF Odometry Estimator 踩坑紀錄

## 1. Kalman Gain 計算：N×N 矩陣求逆 (Run 6)

**問題**：標準 Kalman gain `K = P H^T (H P H^T + R)^{-1}` 需要求逆 N×N 矩陣（N = 有效點數 ~1250），每幀 600–1400ms。

**解法**：Woodbury identity，改為求逆 15×15 矩陣：

```
K = (P^{-1} + H^T R^{-1} H)^{-1} H^T R^{-1}
```

**效果**：solve 從 600–1400ms → 0.1–3ms。

---

## 2. G 矩陣錯誤：I vs -R (Run 10)

**問題**：`G(3,0)` 應為 `-state.R`（acc noise 從 body → world 需旋轉），寫成 `I` 導致 process noise 方向錯誤。

**正確**：
```cpp
G.block<3, 3>(3, 0) = -state.R;  // δv ← acc noise (body→world)
```

---

## 3. Qi 離散化錯誤：σ²Δt² vs σ²Δt (Run 10)

**問題**：連續白噪聲 PSD 離散化應為 `σ²·Δt`，誤寫為 `σ²·Δt²`（多乘一次 Δt）。

**影響**：IMU 高頻 (500Hz, Δt=0.002s) 時，`Δt²` 讓 process noise 被壓到 1/500，P 幾乎不漲→ Kalman gain 太小。

---

## 4. P_bar.inverse() 數值不穩定 (Run 10)

**問題**：直接用 `.inverse()` 對 P_bar 求逆，P 矩陣可能接近奇異。

**解法**：改用 LDLT 分解：
```cpp
P_bar.ldlt().solve(Eigen::Matrix<double, 15, 15>::Identity())
```

---

## 5. imu_buf_ Trim 策略：10 秒滾動窗 vs prev_scan_time (Run 7–8)

**問題**：用固定 10 秒窗口 trim imu_buf_，當 frontend 處理慢時（每幀 >100ms），早期 IMU 數據在被使用前就被丟棄。

**解法**：改為只 trim `prev_scan_time_` 之前的數據，保證 estimateWithFeatures 需要的 IMU 都在。

---

## 6. Deskew 在降採樣前做 (Run 9)

**問題**：先 deskew 26k 點再 VoxelGrid 降到 3k，浪費大量計算。

**解法**：VoxelGrid 先做（26k → 3k），再 deskew 3k 點。

---

## 7. std::cerr + std::endl I/O 背壓 (Run 11)

**問題**：大量 `std::cerr << ... << std::endl` 導致每次 flush。cerr 是 unbuffered，加上 endl 強制 flush，在高頻調用時產生 **2434ms 的 I/O stall**。

**解法**：
- `std::cerr` → `std::clog`（buffered）
- `std::endl` → `'\n'`（不 flush）
- 只保留一行 summary log

---

## 8. repropagate 使用 propagateStep（含 covariance）(Run 11)

**問題**：每幀結束後需 repropagate 未來 IMU 到 predicted_states。用 `propagateStep` 會算 15×15 covariance propagation，2783 個 IMU 樣本花了 **3606ms**。

**解法**：改用 `predictStep`（只算 nominal state，不算 P）。predicted_states 只需 pose/vel 做 deskew，不需要 covariance。

**效果**：3606ms → 0.4ms。

---

## 9. high_resolution_clock NTP 問題 (Run 11)

**問題**：`std::chrono::high_resolution_clock` 在 Linux 上可能受 NTP 調整影響，計時偶爾出現負值或跳變。

**解法**：改用 `std::chrono::steady_clock`。

---

## 10. Gravity 在 State 裡（18-DOF）的可觀性問題 (Run 12–17)

**根因**：State 含 gravity（R³, 3-DOF），但 scan matching H 矩陣裡**沒有直接的 gravity 觀測**。gravity 只能靠 F 矩陣 `∂δv/∂δg = I·dt` 的 P 交叉相關間接更新。

### 10a. 不 apply gravity update (Run 12)

**問題**：`dx_total.segment<3>(15)` 算出來但沒寫回 `gravity_`。Kalman gain 分配了一部分 innovation 給 δg，卻被丟掉 — b_a 的修正量被「漏」給 phantom state。

**Z drift**：-10.3m。

### 10b. Apply gravity update (Run 13)

加了 `gravity_ += dx_total.segment<3>(15)` 後 Z 改善到 -5.5m，但問題沒解決。

### 10c. P_grav 太大 → 爆炸 (Run 14)

`P_grav = 0.01` → b_a_z 飆到 0.9，Z = -552m。gravity magnitude 在 R³ 裡不受約束，|g| 可以自由變化。

### 10d. 根本解法：移除 gravity (Run 15)

改為 15-DOF state，gravity 當常數。**Z = +0.88m，b_a_z = -0.019。**

**教訓**：

| State 設計 | 問題 |
|---|---|
| gravity in R³ (3-DOF) | b_a ↔ g 完全耦合，magnitude 不受約束 |
| gravity in S² (2-DOF) | magnitude 鎖死，但實現複雜（boxplus/boxminus） |
| gravity 常數 (0-DOF) | 最簡單，跟 LIO-SAM 一致，依賴好的初始化 |

---

## 11. 初始化 R₀ 的重要性 (Run 15–17)

### Run 15 的真相

表面上 Z = +0.88m 很好，但 **b_a_x = -0.589**（真實 bias ~0.01）。

原因：首個 IMU `acc = (0.57, -0.52, 8.50)`，車在動，gravity alignment 偏了 ~3.4°。偏差 = `9.8 × sin(3.4°) ≈ 0.58`，被 b_a_x 吸收。

### Run 16: best-of-50 + P_ba=1e-5

鎖死 b_a 不讓它補償 → 真正的 bias 也補不了 → **Z = -85m 飛了**。

### Run 17: best-of-50 + P_θ=0.01 + P_ba=1e-3

|acc| 最接近 g 的樣本方向仍然偏 8–11° → Z = -804m。

**教訓**：

- P tuning **不能解決初始化偏差問題** — P_ba 大→吸 gravity error；P_ba 小→不能補真 bias
- 「車從開始就在動」的 dataset 不能靠單/多 IMU 樣本做 gravity alignment
- 正確做法：multi-frame scan matching + linear alignment（VINS-Mono 風格）

---

## 12. LIO-SAM vs 我們的 iEKF 為何差這麼多

| | LIO-SAM | 我們的 iEKF |
|---|---|---|
| gravity | **常數**，不在 state | 曾在 state (18-DOF)，現為常數 |
| bias 更新 | factor graph 多幀聯合優化 | 只靠 P 交叉相關間接更新 |
| 误差回溯 | iSAM2 可修正歷史 pose + bias | EKF 只改當前幀 |
| 初始化 | 假設靜止（或多幀 IMU 平均） | 單 IMU 樣本（車在動，~4° 偏差） |

---

## 開發時間線 & Run 結果

| Run | Median(ms) | Max(ms) | Z_final | 關鍵改動 |
|-----|-----------|---------|---------|----------|
| 6   | — | 1400 | — | 原始 N×N Kalman |
| 7   | — | — | — | Woodbury (18×18 solve) |
| 8   | — | — | — | imu_buf_ trim fix |
| 9   | — | — | — | Deskew 優化 (downsample first) |
| 10  | — | — | — | G matrix fix, Qi fix, LDLT |
| 11  | 42.9 | 2550 | -10.3 | Timing instrumentation |
| 12  | 29.1 | 98.9 | -10.3 | steady_clock + predictStep + clog |
| 13  | 22.4 | 71.5 | -5.5  | gravity update applied |
| 14  | 34.7 | 144.7 | **-552** | P_grav=0.01 (爆了) |
| 15  | 24.5 | 63.6 | **+0.88** | **15-DOF (gravity 常數)** |
| 16  | 22.0 | 78.0 | -85.7 | 50-sample avg + P_ba=1e-5 |
| 17  | 22.0 | 78.0 | -804 | best-of-50 + P_θ=0.01 |

---

## 下一步：Multi-frame Init

目標：前 N 幀用純 scan-to-map matching 拿 pose，配合 IMU preintegration 做 linear alignment 求解 `{v₀, g, R₀}`。消除對靜止假設的依賴。
