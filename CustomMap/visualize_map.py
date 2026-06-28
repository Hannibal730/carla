#!/usr/bin/env python3
"""
CustomMap 항공뷰(top-down) OpenCV 시각화 + 마우스 좌표 표시.

XODR(OpenDRIVE)을 CARLA로 오프라인 파싱하여 모든 레인(주행/주차칸/인도/갓길)의
중심선과 좌/우 경계를 실제 CARLA 월드 좌표(미터)로 그린다.
마우스를 올리면 해당 픽셀의 CARLA (x, y) 좌표(미터)가 표시된다.

사용:
    python visualize_map.py MandoParking2
    python visualize_map.py CustomMap/MandoParking2/Mando2.xodr
    python visualize_map.py MandoParking2 --step 0.3 --size 1200
"""
import argparse
import glob
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import cv2
import carla

HERE = os.path.dirname(os.path.abspath(__file__))

# 레인 타입별 색상 (BGR)
LANE_COLORS = {
    "driving":  (90, 200, 90),    # 초록
    "parking":  (60, 170, 255),   # 주황
    "shoulder": (120, 120, 120),  # 회색
    "sidewalk": (200, 160, 120),  # 청회색
    "border":   (80, 80, 80),
    "none":     (60, 60, 60),
}
DEFAULT_COLOR = (100, 100, 100)


def resolve_xodr(arg):
    """맵 이름 또는 .xodr 경로를 받아 실제 .xodr 파일 경로를 반환."""
    if arg.endswith(".xodr") and os.path.isfile(arg):
        return arg
    # 맵 이름으로 폴더 검색
    for base in (arg, os.path.join(HERE, arg)):
        if os.path.isdir(base):
            hits = glob.glob(os.path.join(base, "*.xodr"))
            if hits:
                return hits[0]
    # 부분 일치 폴더 검색
    hits = glob.glob(os.path.join(HERE, "*", "*.xodr"))
    for h in hits:
        if arg.lower() in h.lower():
            return h
    raise FileNotFoundError(f"'{arg}' 에 해당하는 .xodr 를 찾지 못했습니다.")


def lane_section_ranges(road):
    """road 의 각 laneSection 의 (s_start, s_end) 리스트."""
    length = float(road.get("length"))
    secs = road.find("lanes").findall("laneSection")
    starts = [float(ls.get("s")) for ls in secs]
    ranges = []
    for i, s0 in enumerate(starts):
        s1 = starts[i + 1] if i + 1 < len(starts) else length
        ranges.append((s0, s1))
    return list(zip(secs, ranges))


def extract_polylines(xodr_path, step):
    """
    XODR 의 모든 레인을 따라가며 (lane_type, center[], left[], right[]) 폴리라인 추출.
    좌표는 CARLA 월드 (x, y) 미터.
    """
    xodr = open(xodr_path).read()
    cmap = carla.Map(os.path.splitext(os.path.basename(xodr_path))[0], xodr)
    root = ET.fromstring(xodr)

    lanes = []
    for road in root.findall("road"):
        rid = int(road.get("id"))
        for sec, (s0, s1) in lane_section_ranges(road):
            for side in ("left", "right"):
                sec_side = sec.find(side)
                if sec_side is None:
                    continue
                for lane in sec_side.findall("lane"):
                    lid = int(lane.get("id"))
                    ltype = lane.get("type")
                    center, left, right = [], [], []
                    s = s0 + 1e-3
                    while s < s1:
                        wp = cmap.get_waypoint_xodr(rid, lid, s)
                        if wp is not None:
                            tf = wp.transform
                            loc = tf.location
                            rv = tf.get_right_vector()
                            hw = wp.lane_width / 2.0
                            center.append((loc.x, loc.y))
                            left.append((loc.x - rv.x * hw, loc.y - rv.y * hw))
                            right.append((loc.x + rv.x * hw, loc.y + rv.y * hw))
                        s += step
                    if len(center) >= 2:
                        lanes.append((ltype, center, left, right))
    return lanes


def extract_objects(xodr_path):
    """
    XODR 의 <objects>/<object> (장애물 등)를 CARLA 월드 (x, y) 미터로 변환.
    각 object 는 (road_id, s, t) 로 정의되므로, 도로 기준선(lane 0) 의
    위치/방향을 구해 world = ref_loc - right_vector * t 로 변환한다.
    반환: [(name, otype, (x, y), width, length, hdg), ...]
    """
    xodr = open(xodr_path).read()
    cmap = carla.Map(os.path.splitext(os.path.basename(xodr_path))[0], xodr)
    root = ET.fromstring(xodr)

    objs = []
    for road in root.findall("road"):
        rid = int(road.get("id"))
        objects = road.find("objects")
        if objects is None:
            continue
        for obj in objects.findall("object"):
            s = float(obj.get("s"))
            t = float(obj.get("t", 0.0))
            # 도로 기준선(lane 0)에서 위치/방향 취득
            wp = cmap.get_waypoint_xodr(rid, 0, s)
            if wp is None:
                continue
            loc = wp.transform.location
            rv = wp.transform.get_right_vector()
            # OpenDRIVE +t = 좌측 → right_vector 의 반대 방향
            wx = loc.x - rv.x * t
            wy = loc.y - rv.y * t
            objs.append((
                obj.get("name", ""),
                obj.get("type", ""),
                (wx, wy),
                float(obj.get("width", 0.0)),
                float(obj.get("length", 0.0)),
                float(obj.get("hdg", 0.0)),
            ))
    return objs


class AerialView:
    def __init__(self, lanes, objects=None, size=1000, margin=40):
        self.lanes = lanes
        self.objects = objects or []
        self.margin = margin
        pts = np.array(
            [p for _, c, l, r in lanes for grp in (c, l, r) for p in grp],
            dtype=np.float64,
        )
        self.xmin, self.ymin = pts.min(axis=0)
        self.xmax, self.ymax = pts.max(axis=0)
        span_x = self.xmax - self.xmin
        span_y = self.ymax - self.ymin
        usable = size - 2 * margin
        self.scale = usable / max(span_x, span_y, 1e-6)
        self.W = int(span_x * self.scale) + 2 * margin
        self.H = int(span_y * self.scale) + 2 * margin
        self.base = self._render()
        self.mouse_world = None

    def world_to_px(self, wx, wy):
        u = int((wx - self.xmin) * self.scale) + self.margin
        v = int((wy - self.ymin) * self.scale) + self.margin
        return u, v

    def px_to_world(self, u, v):
        wx = (u - self.margin) / self.scale + self.xmin
        wy = (v - self.margin) / self.scale + self.ymin
        return wx, wy

    def _poly(self, pts):
        return np.array([self.world_to_px(x, y) for x, y in pts], dtype=np.int32)

    def _render(self):
        img = np.full((self.H, self.W, 3), 30, dtype=np.uint8)
        # 레인 면(중심을 따라 좌→우 경계로 폴리곤) 채우기
        for ltype, center, left, right in self.lanes:
            color = LANE_COLORS.get(ltype, DEFAULT_COLOR)
            ring = self._poly(left + right[::-1])
            cv2.fillPoly(img, [ring], tuple(int(c * 0.35) for c in color))
        # 경계선 + 중심선
        for ltype, center, left, right in self.lanes:
            color = LANE_COLORS.get(ltype, DEFAULT_COLOR)
            cv2.polylines(img, [self._poly(left)], False, color, 1, cv2.LINE_AA)
            cv2.polylines(img, [self._poly(right)], False, color, 1, cv2.LINE_AA)
            if ltype == "driving":
                cv2.polylines(img, [self._poly(center)], False, (255, 255, 0), 1, cv2.LINE_AA)
        self._draw_objects(img)
        self._draw_axes(img)
        self._draw_legend(img)
        return img

    def _draw_objects(self, img):
        # 장애물: 빨간 원(중심) + 외형 박스(width x length, hdg 회전)
        for name, otype, (wx, wy), width, length, hdg in self.objects:
            u, v = self.world_to_px(wx, wy)
            if width > 0 and length > 0:
                hw = width / 2.0
                hl = length / 2.0
                # 로컬 코너 → hdg 회전 → 월드 → 픽셀
                corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
                cos, sin = np.cos(hdg), np.sin(hdg)
                box = []
                for lx, ly in corners:
                    rx = wx + lx * cos - ly * sin
                    ry = wy + lx * sin + ly * cos
                    box.append(self.world_to_px(rx, ry))
                cv2.polylines(img, [np.array(box, dtype=np.int32)], True,
                              (0, 0, 255), 1, cv2.LINE_AA)
            cv2.circle(img, (u, v), 3, (0, 0, 255), -1, cv2.LINE_AA)
            label = name if otype != "obstacle" else f"{name} [obstacle]"
            cv2.putText(img, label, (u + 5, v - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (0, 0, 255), 1, cv2.LINE_AA)

    def _draw_axes(self, img):
        # 10m 그리드
        gx0 = np.ceil(self.xmin / 10) * 10
        gy0 = np.ceil(self.ymin / 10) * 10
        x = gx0
        while x <= self.xmax:
            u, _ = self.world_to_px(x, self.ymin)
            cv2.line(img, (u, self.margin), (u, self.H - self.margin), (55, 55, 55), 1)
            cv2.putText(img, f"{int(x)}", (u - 10, self.H - self.margin + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1, cv2.LINE_AA)
            x += 10
        y = gy0
        while y <= self.ymax:
            _, v = self.world_to_px(self.xmin, y)
            cv2.line(img, (self.margin, v), (self.W - self.margin, v), (55, 55, 55), 1)
            cv2.putText(img, f"{int(y)}", (4, v + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1, cv2.LINE_AA)
            y += 10

    def _draw_legend(self, img):
        y = 18
        for name, col in LANE_COLORS.items():
            cv2.line(img, (self.W - 110, y - 4), (self.W - 90, y - 4), col, 3)
            cv2.putText(img, name, (self.W - 84, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (220, 220, 220), 1, cv2.LINE_AA)
            y += 16

    def on_mouse(self, event, x, y, flags, param):
        self.mouse_world = self.px_to_world(x, y)
        self.mouse_px = (x, y)

    def frame(self):
        img = self.base.copy()
        if self.mouse_world is not None:
            wx, wy = self.mouse_world
            u, v = self.mouse_px
            cv2.line(img, (u, 0), (u, self.H), (0, 0, 255), 1)
            cv2.line(img, (0, v), (self.W, v), (0, 0, 255), 1)
            label = f"x={wx:7.2f}  y={wy:7.2f}  (m, CARLA)"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(img, (8, 8), (8 + tw + 12, 8 + th + 14), (0, 0, 0), -1)
            cv2.putText(img, label, (14, 8 + th + 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2, cv2.LINE_AA)
        return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map", help="맵 이름(예: MandoParking2) 또는 .xodr 경로")
    ap.add_argument("--step", type=float, default=0.5, help="레인 샘플링 간격(m)")
    ap.add_argument("--size", type=int, default=1000, help="최대 창 크기(px)")
    args = ap.parse_args()

    xodr_path = resolve_xodr(args.map)
    print(f"[i] XODR: {xodr_path}")
    lanes = extract_polylines(xodr_path, args.step)
    objects = extract_objects(xodr_path)
    print(f"[i] {len(lanes)} 개 레인, {len(objects)} 개 장애물(object) 추출 완료")
    if not lanes:
        print("[!] 그릴 레인이 없습니다.")
        sys.exit(1)

    view = AerialView(lanes, objects, size=args.size)
    print(f"[i] 범위  x: {view.xmin:.1f}..{view.xmax:.1f}  "
          f"y: {view.ymin:.1f}..{view.ymax:.1f} (m)")
    win = "AerialView - " + os.path.basename(xodr_path)
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, view.on_mouse)
    print("[i] 마우스를 올리면 좌표 표시. q/ESC 로 종료.")
    while True:
        cv2.imshow(win, view.frame())
        k = cv2.waitKey(20) & 0xFF
        if k in (ord("q"), 27):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
