"""COVERAGE-AI backend (single router): UDP RSSI + overhead-camera ArUco localisation -> AP fit + GP -> dashboard.
Run:  python server.py --sim    (no hardware)      |      python server.py    (real bot + cameras)
Open: http://localhost:8000"""
import argparse, asyncio, csv, json, os, socket, threading, time, warnings
import numpy as np, uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse
from scipy.optimize import least_squares
from sklearn.gaussian_process import GaussianProcessRegressor as GPR
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
warnings.filterwarnings("ignore")

# ================= CONFIG (edit me) =================
W, H = 1.5, 1.5                                   # arena size, metres
CAMS = ["http://192.168.1.104:8080", "http://192.168.1.102:8080"]
STATIC = {0: (0.10, 0.10), 1: (1.40, 0.10), 21: (0.10, 0.80), 22: (1.40, 0.80), 2: (0.10, 1.40), 3: (1.40, 1.40)}
BOT_ID = 10
ARUCO_DICT = "DICT_6X6_250"
UDP_PORT, ESP_PORT = 5005, 5006
TARGET_BSSID = "B4:86:18:2C:3D:6E"                               # e.g. "aa:bb:cc:dd:ee:ff" to ignore other APs
CELL, WEAK = 0.05, -75.0                          # grid cell (m), weak-zone threshold (dBm)
MAX_POS_AGE = 0.4                                 # s, max age of a position paired with an RSSI sample
# =====================================================

HERE = os.path.dirname(os.path.abspath(__file__))
S, pos, live, esp = [], {}, {"rssi": None, "bssid": "", "ch": 0}, {"addr": None}
model, force = {"ver": 0, "data": None}, threading.Event()
logf = open(os.path.join(HERE, "log.csv"), "a", newline=""); logw = csv.writer(logf)
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def pos_now():
    t = time.time(); v = [(x, y) for x, y, ts in list(pos.values()) if t - ts < MAX_POS_AGE]
    return tuple(float(a) for a in np.mean(v, axis=0)) if v else None

def add_sample(rssi, bssid="", ch=0):
    live.update(rssi=rssi, bssid=bssid, ch=ch)
    if TARGET_BSSID and bssid.lower() != TARGET_BSSID.lower(): return
    p = pos_now()
    if p is None: return
    S.append([p[0], p[1], rssi]); logw.writerow([time.time(), *p, rssi, bssid, ch]); logf.flush()

def drive(l, r):
    if esp["addr"]: cmd_sock.sendto(f"M,{int(l)},{int(r)}".encode(), (esp["addr"], ESP_PORT))

def udp_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", UDP_PORT))

    print(f"[UDP] Listening on 0.0.0.0:{UDP_PORT}")

    while True:
        d, a = s.recvfrom(1024)

        print("[UDP] RECEIVED:", d)

        try:
            j = json.loads(d.decode("utf-8"))

            live["rssi"] = j["rssi"]
            live["bssid"] = j.get("bssid", "")
            live["ch"] = j.get("ch", 0)

            esp["addr"] = a[0]

            print(
                f"[RSSI] {live['rssi']} dBm | "
                f"BSSID={live['bssid']} | "
                f"CH={live['ch']}"
            )

        except Exception as e:
            print("[UDP ERROR]", e)

def cam_loop(i, src):
    import cv2
    if src.startswith("http") and src.rstrip("/").count("/") < 3: src = src.rstrip("/") + "/video"   # IP Webcam app
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT)),
                                  cv2.aruco.DetectorParameters())
    cap, Hm, ema, t0, fr, seen = None, None, None, time.time(), 0, set()
    while True:
        if cap is None:
            print(f"[cam{i}] connecting {src}"); cap = cv2.VideoCapture(src)
            if not cap.isOpened(): print(f"[cam{i}] cannot open stream (check URL / same network)"); cap = None; time.sleep(2); continue
            print(f"[cam{i}] stream opened")
        ok, f = cap.read()
        if not ok: print(f"[cam{i}] read failed, reconnecting"); cap.release(); cap = None; time.sleep(1); continue
        fr += 1
        corners, ids, _ = det.detectMarkers(f)
        c = {} if ids is None else {int(k): q.reshape(4, 2).mean(0) for k, q in zip(ids.ravel(), corners)}
        seen.update(c)
        if time.time() - t0 > 3:
            print(f"[cam{i}] {fr/3:.0f} fps {f.shape[1]}x{f.shape[0]} | markers seen {sorted(seen)} | "
                  f"homography {'OK' if Hm is not None else 'NO (needs 4 floor markers)'} | bot {'seen' if BOT_ID in seen else 'NOT seen'}")
            t0, fr, seen = time.time(), 0, set()
        st = [k for k in c if k in STATIC]
        if len(st) >= 4:
            Hm, _ = cv2.findHomography(np.float32([c[k] for k in st]), np.float32([STATIC[k] for k in st]))
        if Hm is not None and BOT_ID in c:
            x, y = cv2.perspectiveTransform(c[BOT_ID].reshape(1, 1, 2).astype(np.float32), Hm)[0, 0]
            ema = (x, y) if ema is None else (.6 * ema[0] + .4 * x, .6 * ema[1] + .4 * y)
            pos[i] = (ema[0], ema[1], time.time())

def sim_loop():   # fake bot + hidden AP
    ap = np.array([.4, 1.1]); x, y, dx = .1, .1, .01
    print("SIM: true AP at", ap.tolist())
    while True:
        x += dx
        if x > W - .1 or x < .1: dx = -dx; y = y + .15 if y + .15 < H - .1 else .1
        d = np.hypot(x - ap[0], y - ap[1]) + .1
        pos["sim"] = (x, y, time.time()); esp["addr"] = "sim"
        add_sample(int(round(-35 - 26 * np.log10(d) + np.random.randn() * 2.5)), "sim", 6)
        time.sleep(.05)

# ---------- model ----------
def pl(p, X):
    d = np.hypot(X[:, 0] - p[0], X[:, 1] - p[1]) + .1
    return p[2] - 10 * p[3] * np.log10(d)

def place(weak):
    p, G, mu, gx, gy = (model[k] for k in ("p", "G", "mu", "gx", "gy"))
    base = (mu < weak).sum()
    if base == 0: return None
    best = (base, None)
    for b in gy[::3]:
        for a in gx[::3]:
            w = (np.maximum(mu, pl([a, b, p[2], p[3]], G)) < weak).sum()
            if w < best[0]: best = (w, (a, b))
    if best[1] is None: return None
    return {"x": round(float(best[1][0]), 2), "y": round(float(best[1][1]), 2),
            "before": round(100 * base / len(mu), 1), "after": round(100 * best[0] / len(mu), 1)}

def publish():
    m = model; ny, nx = len(m["gy"]), len(m["gx"]); A = np.array(S)
    m["data"] = {"type": "map", "gx": m["gx"].round(3).tolist(), "gy": m["gy"].round(3).tolist(),
                 "mu": m["mu"].reshape(ny, nx).round(1).tolist(), "sd": m["sd"].reshape(ny, nx).round(2).tolist(),
                 "ap": m["p"][:2].round(2).tolist(), "p0": round(float(m["p"][2]), 1), "n": round(float(m["p"][3]), 2),
                 "rmse": round(float(m["rmse"]), 2), "rep": place(m.get("weak", WEAK)),
                 "pts": A[::max(1, len(A) // 500)].round(2).tolist()}
    m["ver"] += 1

def train():
    A = np.array(S); k = np.floor(A[:, :2] / CELL).astype(int)
    _, inv = np.unique(k, axis=0, return_inverse=True); inv = inv.ravel(); cnt = np.bincount(inv)
    X = np.c_[np.bincount(inv, A[:, 0]) / cnt, np.bincount(inv, A[:, 1]) / cnt]; y = np.bincount(inv, A[:, 2]) / cnt
    if len(X) > 600: i = np.random.choice(len(X), 600, replace=False); X, y = X[i], y[i]
    best = None
    for s in [X[y.argmax()]] + [np.random.rand(2) * [W, H] for _ in range(6)]:
        r = least_squares(lambda p: pl(p, X) - y, [s[0], s[1], -40, 2.5], loss="soft_l1", f_scale=3,
                          bounds=([-W, -H, -100, 1], [2 * W, 2 * H, 0, 6]))
        if best is None or r.cost < best.cost: best = r
    p = best.x
    gp = GPR(C(10) * RBF(.6, (.1, 5)) + WhiteKernel(4), n_restarts_optimizer=1).fit(X, y - pl(p, X))
    gx, gy = np.arange(CELL / 2, W, CELL), np.arange(CELL / 2, H, CELL)
    G = np.array([[a, b] for b in gy for a in gx]); r, sd = gp.predict(G, return_std=True)
    model.update(p=p, G=G, mu=pl(p, G) + r, sd=sd, gx=gx, gy=gy, rmse=np.sqrt(np.mean((pl(p, X) - y) ** 2)))
    publish()

def trainer():
    last = 0
    while True:
        time.sleep(1); n = len(S)
        if n < last: last = 0
        if n >= 20 and (n - last >= 25 or force.is_set()):
            force.clear(); last = n
            try: train()
            except Exception as e: print("train error:", e)

# ---------- web ----------
app = FastAPI()

@app.get("/")
def index(): return FileResponse(os.path.join(HERE, "static", "dashboard.html"))

@app.get("/export")
def export(): return FileResponse(os.path.join(HERE, "log.csv"), filename="coverage_log.csv")

@app.websocket("/ws")
async def ws(w: WebSocket):
    await w.accept(); sent = 0
    async def tx():
        nonlocal sent
        while True:
            await w.send_json({"type": "live", "rssi": live["rssi"], "bssid": live["bssid"], "ch": live["ch"],
                               "pos": pos_now(), "n": len(S), "esp": esp["addr"], "size": [W, H]})
            if model["ver"] != sent and model["data"]: sent = model["ver"]; await w.send_json(model["data"])
            await asyncio.sleep(.2)
    async def rx():
        while True:
            m = await w.receive_json(); c = m.get("cmd")
            if c == "drive": drive(m["l"], m["r"])
            elif c == "retrain": force.set()
            elif c == "clear": S.clear(); model["data"] = {"type": "clear"}; model["ver"] += 1; model.pop("p", None)
            elif c == "weak":
                model["weak"] = float(m["v"])
                if "p" in model: publish()
    try: await asyncio.gather(tx(), rx())
    except Exception: pass

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--sim", action="store_true"); a = ap.parse_args()
    threading.Thread(target=trainer, daemon=True).start()
    if a.sim: threading.Thread(target=sim_loop, daemon=True).start()
    else:
        threading.Thread(target=udp_loop, daemon=True).start()
        for i, s in enumerate(CAMS): threading.Thread(target=cam_loop, args=(i, s), daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
