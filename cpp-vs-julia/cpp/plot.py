import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルのパス
run_dir = "run/"
num_time_steps = 1000
time_step = 0.1


# CSVファイルを読み込む関数
def read_csv(file_path):
    # 空のフィールドや不適切なデータを無視する
    try:
        return np.loadtxt(file_path, delimiter=",")
    except ValueError as e:
        print(f"Error reading {file_path}: {e}")
        return np.zeros((150, 150))


# アニメーションを作成する関数
def create_animation(output_dir, num_time_steps):
    fig, ax = plt.subplots()
    ax.set_title("Temperature Distribution")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    cax = ax.imshow(
        read_csv(os.path.join(output_dir, "output_0.csv")),
        cmap="inferno",
        interpolation="nearest",
        origin="lower",
        vmin=0,
        vmax=10,
        aspect="auto",
    )
    fig.colorbar(cax)
    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes, color="white")

    def update(frame):
        data = read_csv(os.path.join(output_dir, f"output_{frame}.csv"))
        cax.set_array(data)
        time_text.set_text(f"Time = {frame * time_step:.1f}s")
        return cax, time_text

    ani = animation.FuncAnimation(fig, update, frames=num_time_steps, blit=True)

    # 保存する場合
    ani.save("heat_equation_2d.mp4", fps=10)


# メイン関数
def main():
    create_animation(run_dir, num_time_steps)


if __name__ == "__main__":
    main()
