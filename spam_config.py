import pandas as pd
import matplotlib.pyplot as plt
import subprocess
import numpy as np
from datetime import datetime
import numpy as np

def permutation_sequence_3():
    char = ["P", "R"]
    return ["".join(p) for p in list(np.array(np.meshgrid(char, char)).T.reshape(-1, 2))]

def get_current_time() -> str:
    return datetime.now().strftime("%Y%m%d%H%M%S%f")

subprocess.run(["g++", "-o", "main", "main.cpp", "-lm"])

iteration = 100
sequence = permutation_sequence_3()
ROOT_FOLDER = "package"

# Empty img folder
import os

for sequence_name in sequence:
    image_folder = os.path.join(ROOT_FOLDER, sequence_name, "img")
    report_file = os.path.join(ROOT_FOLDER, sequence_name, "report.md")

    os.makedirs(image_folder, exist_ok=True)
    for filename in os.listdir(image_folder):
        os.remove(os.path.join(image_folder, filename))

    # Empty report.md
    with open(report_file, "w") as f:
        f.write("")

    for _ in range(iteration):
        subprocess.run(["main.exe", sequence_name])
        series_name = get_current_time()

        df = pd.read_csv("trajectory.csv")
        headers = df.columns.tolist()[:-3] # Last 3 columns are time, x, y, 2

        # Compute global axis limits for uniform scaling
        x = df[headers[0]]
        y = df[headers[1]]
        z = df[headers[2]]

        x_range = [x.min(), x.max()]
        y_range = [y.min(), y.max()]
        z_range = [z.min(), z.max()]

        # Set up subplots
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        (ax_xy, ax_xz), (ax_yz, _) = axes  # last one we'll replace with 3D

        # XY
        ax_xy.plot(x, y, color='blue', label='Trajectory')
        ax_xy.scatter(x.iloc[0], y.iloc[0], color='green', label='Start')
        ax_xy.scatter(x.iloc[-1], y.iloc[-1], color='red', label='End')
        ax_xy.set_title("XY Plane View")
        ax_xy.set_xlabel("X (m)")
        ax_xy.set_ylabel("Y (m)")
        ax_xy.legend()
        ax_xy.grid(True)

        # XZ
        ax_xz.plot(x, z, color='blue', label='Trajectory')
        ax_xz.scatter(x.iloc[0], z.iloc[0], color='green', label='Start')
        ax_xz.scatter(x.iloc[-1], z.iloc[-1], color='red', label='End')
        ax_xz.set_title("XZ Plane View")
        ax_xz.set_xlabel("X (m)")
        ax_xz.set_ylabel("Z (m)")
        ax_xz.legend()
        ax_xz.grid(True)

        # YZ
        ax_yz.plot(y, z, color='blue', label='Trajectory')
        ax_yz.scatter(y.iloc[0], z.iloc[0], color='green', label='Start')
        ax_yz.scatter(y.iloc[-1], z.iloc[-1], color='red', label='End')
        ax_yz.set_title("YZ Plane View")
        ax_yz.set_xlabel("Y (m)")
        ax_yz.set_ylabel("Z (m)")
        ax_yz.legend()
        ax_yz.grid(True)

        # Replace bottom-right with 3D subplot
        ax_3d = fig.add_subplot(2, 2, 4, projection='3d')
        ax_3d.plot(x, y, z, color='blue', label='Trajectory')
        ax_3d.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='green', label='Start')
        ax_3d.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='red', label='End')
        ax_3d.set_title("3D Trajectory View")
        ax_3d.set_xlabel("X (m)")
        ax_3d.set_ylabel("Y (m)")
        ax_3d.set_zlabel("Z (m)")
        ax_3d.legend()
        ax_3d.grid(True)

        # Show all
        plt.tight_layout()
        plt.savefig(os.path.join(image_folder, f"iteration_{series_name}.png"), dpi=300)
        plt.close(fig)

        with open(report_file, "a") as f:
            f.write(f"\n![Trajectory](img/iteration_{series_name}.png)\n\n")