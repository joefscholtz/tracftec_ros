#!/usr/bin/python3

import pandas as pd
from pathlib import Path

current_path = Path(__file__).parent.resolve()

df = pd.read_csv(current_path / "origins.csv", header=0)
cols = ["x", "y", "z"]

# Convert mm to meters
for col in cols:
    df[col] = df[col] / 1000.0

print(df)

# Combine caster transforms into single left_caster and right_caster
# using *_caster_pivot positions (ignoring *_caster_body)
left_caster_pivot = df[df["mesh"] == "left_caster_pivot"][["x", "y", "z"]].iloc[0]
right_caster_pivot = df[df["mesh"] == "right_caster_pivot"][["x", "y", "z"]].iloc[0]

# Specify the print order
print_order = [
    ("body", "base"),
    ("left_wheel", "body"),
    ("right_wheel", "body"),
    ("left_caster", "body"),
    ("right_caster", "body"),
    ("mid360", "body"),
    ("imu_sensor", "mid360"),  # fixed
]

# Generate and print <xacro> blocks
for mesh_name, parent in print_order:
    if mesh_name == "left_caster":
        # Relative transform from body -> left_caster_pivot
        x = round(left_caster_pivot["x"] - df[df["mesh"] == parent]["x"].iloc[0], 4)
        y = round(left_caster_pivot["y"] - df[df["mesh"] == parent]["y"].iloc[0], 4)
        z = round(left_caster_pivot["z"] - df[df["mesh"] == parent]["z"].iloc[0], 4)
    elif mesh_name == "right_caster":
        # Relative transform from body -> right_caster_pivot
        x = round(right_caster_pivot["x"] - df[df["mesh"] == parent]["x"].iloc[0], 4)
        y = round(right_caster_pivot["y"] - df[df["mesh"] == parent]["y"].iloc[0], 4)
        z = round(right_caster_pivot["z"] - df[df["mesh"] == parent]["z"].iloc[0], 4)
    elif mesh_name == "imu_sensor":
        # IMU sensor fixed at origin relative to mid360
        x, y, z = 0.0, 0.0, 0.0
    else:
        # Compute relative transform from parent to mesh_name
        mesh_pos = df[df["mesh"] == mesh_name][["x", "y", "z"]].iloc[0]
        parent_pos = df[df["mesh"] == parent][["x", "y", "z"]].iloc[0]
        x = round(mesh_pos["x"] - parent_pos["x"], 4)
        y = round(mesh_pos["y"] - parent_pos["y"], 4)
        z = round(mesh_pos["z"] - parent_pos["z"], 4)

    # Print xacro block
    if mesh_name == "imu_sensor":
        print(
            f'  <xacro:imu_sensor name="livox_imu" parent="{parent}" topic="mid360_imu">\n'
            f'    <origin xyz="{x} {y} {z}" rpy="0 0 0"/>\n'
            f"  </xacro:imu_sensor>\n"
        )
    else:
        print(
            f'  <xacro:{mesh_name} name="{mesh_name}" parent="{parent}">\n'
            f'    <origin xyz="{x} {y} {z}" rpy="0 0 0"/>\n'
            f"  </xacro:{mesh_name}>\n"
        )
# === PRINT JOINTS FOR CASTER WHEELS ===
# Compute relative transforms: caster_pivot -> caster_wheel
for caster in ["left", "right"]:
    pivot_pos = df[df["mesh"] == f"{caster}_caster_pivot"][["x", "y", "z"]].iloc[0]
    wheel_pos = df[df["mesh"] == f"{caster}_caster_wheel"][["x", "y", "z"]].iloc[0]

    rel_x = round(wheel_pos["x"] - pivot_pos["x"], 4)
    rel_y = round(wheel_pos["y"] - pivot_pos["y"], 4)
    rel_z = round(wheel_pos["z"] - pivot_pos["z"], 4)

    print(
        f"{caster}: \n"
        f"\n"
        f'  <joint name="${{name}}_wheel_joint" type="continuous">\n'
        f'    <parent link="${{name}}_pivot"/>\n'
        f'    <child link="${{name}}_wheel"/>\n'
        f'    <origin xyz="{rel_x} {rel_y} {rel_z}" rpy="0 0 0"/>\n'
        f'    <axis xyz="1 0 0"/>\n'
        f"  </joint>\n"
    )
