# Limelight 3A — FTC DECODE (2025-26) Vision Setup Report

**Device:** Limelight 3A (2025.1 firmware) · **Hostname:** `limelight-sfr` (activates on next power-cycle, i.e. when remounted on the Control Hub) · **Networking:** default FTC/USB mode (USB Index 0, Control Hub IP 172.29.0.1) — no static IP set. JSON API on `:5807`, stream on `:5800`, UI on `:5801`.

Configured and live-verified 2026-07-11 with the camera USB-connected to a laptop. Pipeline backups in this folder re-import directly through the Limelight web UI.

## 1. Pipeline configuration

| Idx | Type | Purpose | Key tuned values |
|-----|------|---------|------------------|
| 0 | AprilTag (fiducial) | Goal tracking + MegaTag localization (`Vision.java`) | Family 36h11 (U-Michigan engine); marker size 165.1 mm; ID filter `24, 20`; Full 3D = Yes; snap-to-floor = Yes; **camera-pose fields all 0 (placeholder — must measure on robot)**; field map loaded ✓ |
| 1–2 | (unused) | — | default |
| 3 | Color blob "PurpleBall" | Purple ~5″ ball (`ColorTracker`) | Exposure **1737 (locked)**, gain 28.3; Hue 120–150, Sat 140–255, Val 60–255; erosion 1, dilation 4; area 0.001–100%; sort = Largest |
| 4 | Color blob "GreenBall" | Green ~5″ ball (`ColorTracker`) | Exposure **1600 (locked)**, gain 18; Hue 50–80, Sat 90–255, Val 70–255; erosion 1, dilation 6; area 0.001–100%; sort = Largest |
| 5 | (empty, intentional) | Robot detector — see §3 | default |
| 6–7 | (unused) | — | default |

The robot code's thresholds — "arrived" at `ta >= 3.5%` (`ColorTracker.STOP_AREA_THRESHOLD`) and detector confidence `>= 0.5` (`LimelightObstacleSource.MIN_CONFIDENCE`) — live in robot code, not on the camera.

## 2. Field map / MegaTag status — RESOLVED

The device's loaded field map localizes tags 20 and 24 correctly (green checkmark, 3D visualizer renders both).

**Post-setup finding that closes the one open question:** the official `ftc2025DECODE.fmap` (copy committed in this folder) contains **only tags 20 and 24** — the goal tags, 165.1 mm, 36h11, at mirrored positions (±1.41 m, −1.48 m, 0.75 m high) on a center-origin field. The obelisk tags (21–23) are **not part of the official field map at all**, so the on-device map showing exactly 20 and 24 matches the official file. Nothing is missing.

**MegaTag2 nuance:** `botpose_MT2` (what `Vision.java` reads) requires the robot to stream its yaw to the camera — that only happens with robot code running. Bench verification used MegaTag1: with tag 24 in view, MT1 botpose was stable and sane (WPI-blue ≈ [7.885, 3.679] m, yaw ≈ 90°, tagcount 1, avg distance ≈ 1.91 m, low stdev). MT2 returning near-zero on the bench is expected, not a failure.

## 3. Pipeline 5 (robot detector)

Hardware-capable — a **Hailo neural accelerator is present** (`hailoCount: 1`) — but no robot-detection model exists among the official Limelight models (they are all game-piece detectors). Pipeline left **empty intentionally**: the obstacle-avoidance code (`LimelightObstacleSource`, `DETECTOR_PIPELINE = -1`) treats any detection on its pipeline as a real obstacle, so a placeholder model would create phantom obstacles. Requires a custom-trained detector model upload before use.

## 4. Placeholders / must re-do on the assembled robot

1. **Camera pose in robot space** — all six fields (LL Forward/Right/Up/Roll/Pitch/Yaw) are 0. Measure and enter on the robot; MegaTag field-space accuracy depends on it.
2. **HSV thresholds (pipelines 3–4)** — tuned under bench lighting; exposure is locked to reduce drift, but re-verify under field lighting.
3. **`ta` arrival threshold** — purple peaked >4% and green ≈4.9% on the bench; re-check `ta` at the real approach distance vs the 3.5% threshold.
4. **Pipeline 5** — needs a custom-trained robot-detection model (Hailo-compatible) before `DETECTOR_PIPELINE` is set in code.

## 5. Live verification evidence

- **P0:** real tag 24 in view → 3D cube rendered with "24" label; MT1 botpose stable (§2).
- **P3:** purple ball tracked as a single clean blob; `ta` peaked >4%; rejected red background and green ball.
- **P4:** green ball tracked with tight bounding box; `ta` ≈ 4.9–5.0%; `tx`/`ty` updated correctly with motion; rejected purple.
- **P5:** confirmed default/empty — zero detections generated.

## Files in this folder

| File | What it is |
|------|------------|
| `AprilTagIdentification.vpr` | Pipeline 0 backup (re-importable) |
| `PurpleBall.vpr` | Pipeline 3 backup |
| `GreenBall.vpr` | Pipeline 4 backup |
| `ftc2025DECODE.fmap` | Official DECODE field map (tags 20 + 24) |
