#!/usr/bin/env python3

import math

def parse_scan_log(file_path):
    scans = []
    current_ranges = []
    timestamp = None
    sec = None
    nsec = None

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("sec:"):
                sec = int(line.split(":")[1].strip())
            elif line.startswith("nsec:"):
                nsec = int(line.split(":")[1].strip())
            elif line.startswith("ranges:"):
                try:
                    r = float(line.split(":")[1].strip())
                    current_ranges.append(r)
                except:
                    pass
            elif line == "":  # blank line
                continue
            elif line.startswith("header {") or line.startswith("frame:"):
                # New message starts
                if current_ranges and sec is not None and nsec is not None:
                    timestamp = sec + nsec * 1e-9
                    scans.append({"time": timestamp, "ranges": current_ranges})
                    current_ranges = []
                    sec = None
                    nsec = None

        # Save last scan if file ends
        if current_ranges and sec is not None and nsec is not None:
            timestamp = sec + nsec * 1e-9
            scans.append({"time": timestamp, "ranges": current_ranges})

    return scans

def compute_min_max_distances(scan_data):
    min_dist = float('inf')
    max_dist = 0.0
    for scan in scan_data:
        valid_ranges = [r for r in scan["ranges"] if math.isfinite(r) and r > 0]
        if valid_ranges:
            cur_min = min(valid_ranges)
            cur_max = max(valid_ranges)
            if cur_min < min_dist:
                min_dist = cur_min
            if cur_max > max_dist:
                max_dist = cur_max
    return min_dist, max_dist

if __name__ == "__main__":
    log_file = '/home/ruben/scan.log'  # change if needed

    scans = parse_scan_log(log_file)
    print(f"Parsed {len(scans)} LaserScan records")

    if scans:
        min_d, max_d = compute_min_max_distances(scans)
        print(f"Minimum obstacle distance: {min_d:.3f} meters")
        print(f"Maximum obstacle distance: {max_d:.3f} meters")
    else:
        print("No scan data found")
