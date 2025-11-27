import matplotlib.pyplot as plt
import pandas as pd
import argparse


def load_csv(filename):
    df = pd.read_csv(filename)

    # FIX: strip leading/trailing spaces in column names
    df.columns = df.columns.str.strip()

    # Drop completely empty columns
    df = df.loc[:, df.columns != ""]

    # Normalize time
    df["t"] = df["stamp"] - df["stamp"].iloc[0]

    return df


def plot_xy(df, label, ax):
    ax.plot(df["odom_x"], df["odom_y"], label=f"Odom", linestyle="--")
    ax.plot(df["pf_x"], df["pf_y"], label=f"PF")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)


def plot_theta(df, label, ax):
    ax.plot(df["t"], df["odom_th"], label=f"Odom θ", linestyle="--")
    ax.plot(df["t"], df["pf_th"], label=f"PF θ")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("θ (rad)")
    ax.grid(True)

def main(filenames):
    dfs = [load_csv(f) for f in filenames]
    labels = [f"" for i in range(len(filenames))]

    # ---------- Section 1: XY Plots ----------
    fig1, ax1 = plt.subplots(figsize=(8, 6))
    for df, label in zip(dfs, labels):
        plot_xy(df, label, ax1)
    ax1.set_title("SECTION 1: XY Position — PF vs Odom")
    ax1.legend()
    plt.savefig(filenames[0][:-4]+"_xy.svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
    plt.show()

    # ---------- Section 2: Theta vs Time ----------
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    for df, label in zip(dfs, labels):
        plot_theta(df, label, ax2)
    ax2.set_title("SECTION 2: Theta over Time — PF vs Odom")
    ax2.legend()
    plt.savefig(filenames[0][:-4]+"_theta.svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
    plt.show()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--files", nargs="+", required=True)
    args = parser.parse_args()
    main(args.files)
