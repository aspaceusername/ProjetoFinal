#!/usr/bin/env python3

import pandas as pd
import math

def parse_odom_log(file_path):
    data = []
    with open(file_path, 'r') as f:
        timestamp = None
        x = None
        y = None
        for line in f:
            line = line.strip()
            if line.startswith('sec:'):
                sec = int(line.split(':')[1].strip())
            elif line.startswith('nsec:'):
                nsec = int(line.split(':')[1].strip())
                timestamp = sec + nsec * 1e-9
            elif line.startswith('x:'):
                try:
                    x = float(line.split(':')[1].strip())
                except:
                    x = None
            elif line.startswith('y:'):
                try:
                    y = float(line.split(':')[1].strip())
                except:
                    y = None
            if timestamp is not None and x is not None and y is not None:
                data.append({'time': timestamp, 'x': x, 'y': y})
                timestamp = None
                x = None
                y = None

    return pd.DataFrame(data)

def compute_distance(df):
    df['dx'] = df['x'].diff().fillna(0)
    df['dy'] = df['y'].diff().fillna(0)
    df['dist'] = (df['dx']**2 + df['dy']**2).apply(math.sqrt)
    # Filter out small distances (noise)
    df.loc[df['dist'] < 1e-5, 'dist'] = 0
    total_distance = df['dist'].sum()
    elapsed_time = df['time'].iloc[-1] - df['time'].iloc[0]
    return total_distance, elapsed_time


if __name__ == "__main__":
    log_file = '/home/ruben/odom.log'  # change this to your actual log path
    df = parse_odom_log(log_file)
    print(f"Parsed {len(df)} odometry records")
    if len(df) > 1:
        dist, elapsed = compute_distance(df)
        print(f"Distance traveled: {dist:.3f} meters")
        print(f"Elapsed time: {elapsed:.3f} seconds")
    else:
        print("Not enough data to compute distance")
