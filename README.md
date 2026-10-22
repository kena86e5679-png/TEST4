# 空間偏好 AR：多人連機與標記定位版 (v3)

用手機後鏡頭看真實空間，套用「中央區依移動速度動態縮放、外圍黑底白點依視線掃描記憶決定密度」的視野規則，並支援 **2～3 人連機同步、手持手機敵人偵測與遮擋剔除、Canny 邊緣空間標記物（A-1, A-2...）定位與伺服器即時地圖監控**。

---

## 專案檔案結構（可直接上傳 GitHub）

```text
├── index.html       # 手機端 AR 主程式入口（含 COCO-SSD 視覺模型與連機介面）
├── app.js           # 核心邏輯（後台空間偵測、Canny標記物、敵人遮擋、掃描記憶點雲、雙模連機）
├── dashboard.html   # 伺服器端即時地圖監控儀表板（顯示玩家移動座標與已激活的 A-1, A-2 標記物）
├── server.js        # Node.js + WebSocket 多人連機與地圖定位伺服器
├── package.json     # Node.js 專案相依套件設定
├── .gitignore       # Git 忽略清單
└── README.md        # 專案說明文件
```

---

## 部署方式一：直接上傳到 GitHub Pages（免架設後端也能多人連機！）

為了讓你上傳到 GitHub Pages 就能直接在手機上用 `https://` 測試多人連機，程式內建了 **雙模式網路層（WebSocket + WebRTC PeerJS 自動切換）**：

1. 在 GitHub 建立一個新的 Repository（例如 `ar-vision`），設為 **Public**。
2. 將本專案所有檔案（`index.html`, `app.js`, `dashboard.html`, `server.js`, `package.json`, `README.md`, `.gitignore`）上傳到 Repository 根目錄。
3. 進入 GitHub Repository 的 **Settings → Pages**，在 **Build and deployment → Source** 選擇 `Deploy from a branch`，Branch 選 `main`、資料夾選 `/ (root)`，點擊 **Save**。
4. 等待 1～2 分鐘後即可開啟：
   - **手機端 AR 畫面**：`https://你的帳號.github.io/ar-vision/`
   - **電腦端伺服器地圖監控**：`https://你的帳號.github.io/ar-vision/dashboard.html`
5. **多人連機方式**：2～3 台手機與電腦端 `dashboard.html` 只要輸入相同的「**房間頻道**」（預設為 `AR-ROOM-1`），即可直接跨設備同步座標、顯示敵人與激活標記物！

---

## 部署方式二：架設獨立 Node.js 伺服器（本地或 Render / Railway 雲端主機）

如果你在筆電或雲端伺服器（如 Render.com、Railway）運行 `server.js`，前端會自動優先使用 WebSocket 伺服器進行同步：

```bash
# 1. 安裝相依套件
npm install

# 2. 啟動連機與地圖定位伺服器
npm start
```

- 玩家連線網址：`http://localhost:3000`
- 伺服器即時地圖監控：`http://localhost:3000/dashboard.html`

---

## v3 核心更新功能一覽

| 功能模組 | 實作細節 |
| --- | --- |
| **1. 後台空間偵測與測量** | 所有空間分割、垂直線、地板測距與特徵點運算全數移至背景執行，不在使用者介面上顯示雜亂的青色測量線條。 |
| **2. 敵人偵測與遮擋剔除** | 透過鏡頭與 COCO-SSD 即時捕捉人物與手持手機姿態，識別為連機玩家（敵人）並以**白色方框**標示；當敵人與玩家之間隔著真實牆柱或**虛擬生成的建築**時，自動執行 Ray-Casting 遮擋剔除並隱藏白框。 |
| **3. Canny 邊緣標記物定位** | 結合 **Canny Edge Detector**（高斯濾波、非極大值抑制 NMS、雙閾值遲滯）與表面深度射線，自動在物體邊緣貼附生成編號標記物（`A-1`, `A-2`...）。玩家必須走近至 **2.2 公尺內** 才能激活標記物，同步回傳伺服器擴張地圖並校正步伐漂移。 |
| **4. 掃描記憶點雲密度** | 建立 360°×180° 球面記憶網格，外圍黑底區域的白色點雲密度取決於玩家轉動時「中央清晰視野區」掃描過該方位的累積次數。 |
| **5. 速度動態視野縮放** | 中央視野區依據實際移動速度動態縮放：**靜止不動時最大（1.28x）**、**慢速移動時放大（1.02x～1.28x）**、**快速移動時劇烈縮小（最低至 0.38x）**。 |
