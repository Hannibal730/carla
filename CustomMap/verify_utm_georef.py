#!/usr/bin/env python3
"""
오프라인 xodr georeference == 라이브 CARLA 서버 맵 georeference 검증.

visualize_map.py 가 (xodr 만으로) 계산하는 UTM 이 라이브 GNSS 파이프라인
(/carla/car/f9r/fix → f9r_to_utm → /f9r_utm)과 동일한지 확인한다.

원리:
  라이브 GNSS 센서는 "서버가 로드한 맵"의 georeference 로 lat/lon 을 만든다.
  visualize_map 은 "xodr 로 오프라인 생성한 carla.Map"의 georeference 를 쓴다.
  두 맵의 transform_to_geolocation 이 같으면 → 오프라인 UTM 이 라이브와 동일.

사용 (CARLA 서버가 떠 있어야 함):
    python verify_utm_georef.py MandoParking2
    python verify_utm_georef.py MandoParking2 --host 127.0.0.1 --port 2000
"""
import argparse
import sys

import carla

import visualize_map as vm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map", help="맵 이름(예: MandoParking2) 또는 .xodr 경로")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tol", type=float, default=0.05,
                    help="UTM 허용 오차(m). 이 값 이내면 PASS")
    args = ap.parse_args()

    # 1) 오프라인 맵 (xodr 만으로)
    xodr_path = vm.resolve_xodr(args.map)
    xodr = open(xodr_path).read()
    offline_map = carla.Map("offline", xodr)
    print(f"[i] 오프라인 XODR: {xodr_path}")

    # 2) 라이브 서버 맵
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()
        server_map = world.get_map()
    except Exception as exc:  # noqa: BLE001
        print(f"[!] CARLA 서버 연결 실패: {exc}")
        print("    서버를 먼저 실행하고 맵을 로드한 뒤 다시 시도하세요.")
        sys.exit(2)
    print(f"[i] 라이브 서버 맵: {server_map.name}")

    # 3) 맵 경계 안에서 테스트 점 격자 생성 (xodr header bounds 사용)
    import xml.etree.ElementTree as ET
    hdr = ET.fromstring(xodr).find("header")
    west = float(hdr.get("west")); east = float(hdr.get("east"))
    south = float(hdr.get("south")); north = float(hdr.get("north"))
    pts = []
    for i in range(5):
        for j in range(5):
            x = west + (east - west) * i / 4.0
            y = south + (north - south) * j / 4.0
            pts.append((x, y))

    # 4) 두 맵의 lat/lon · UTM 비교
    max_dlat = max_dlon = 0.0
    max_de = max_dn = 0.0
    for x, y in pts:
        loc = carla.Location(x=x, y=y, z=0.0)
        g_off = offline_map.transform_to_geolocation(loc)
        g_srv = server_map.transform_to_geolocation(loc)
        max_dlat = max(max_dlat, abs(g_off.latitude - g_srv.latitude))
        max_dlon = max(max_dlon, abs(g_off.longitude - g_srv.longitude))
        e_off, n_off, _ = vm.to_utm(g_off.latitude, g_off.longitude)
        e_srv, n_srv, _ = vm.to_utm(g_srv.latitude, g_srv.longitude)
        max_de = max(max_de, abs(e_off - e_srv))
        max_dn = max(max_dn, abs(n_off - n_srv))

    print(f"[i] 테스트 점 {len(pts)} 개 비교")
    print(f"    최대 lat 차이: {max_dlat:.3e} deg")
    print(f"    최대 lon 차이: {max_dlon:.3e} deg")
    print(f"    최대 UTM E 차이: {max_de:.4f} m")
    print(f"    최대 UTM N 차이: {max_dn:.4f} m")

    if max(max_de, max_dn) <= args.tol:
        print(f"[✓] PASS — 두 georeference 가 일치(≤{args.tol} m). "
              f"visualize_map 의 UTM 은 라이브 GNSS 와 동일하게 사용 가능.")
        sys.exit(0)
    else:
        print(f"[✗] FAIL — georeference 불일치(>{args.tol} m). "
              f"서버가 다른 맵/georeference 를 로드했는지 확인 필요.")
        sys.exit(1)


if __name__ == "__main__":
    main()
