"""
plot_errors.py

This module provides functionality to plot error data from CSV files.
It reads error data, processes timestamps, and generates various plots for
P and PID controllers: error/edot vs time, x/y/theta vs time, x vs y, error vs edot.
Each plot includes titles, axis labels, legends, different shapes/colors, and grids.
"""

import matplotlib.pyplot as plt
from utilities import FileReader


def plot_e_edot_t(filenames):
    """
    Plots error (e) and error_dot (edot) over time (t) for multiple files.

    Args:
        filenames (list): List of CSV file paths (e.g., angular_errors.csv for P and PID).
    """
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    markers = ['o', 's', '^', 'D', 'v']
    
    label_base = None
    for idx, filename in enumerate(filenames):
        fig, axs = plt.subplots(1,1,figsize=(10, 6))
        headers, values = FileReader(filename).read_file()
        time_list = [val[-1] - values[0][-1] for val in values]
        if time_list[0] > 1e18:
            time_list = [val / 1e9 for val in time_list]
        error = [val[0] for val in values]  # error
        error_dot = [val[1] for val in values]  # error_dot
        label_base = filename.split('/')[-1].replace('_errors.csv', '').replace('angular', 'Angular').replace('linear', 'Linear')
        axs.plot(time_list, error, color=colors[idx % len(colors)], marker=markers[idx % len(markers)], markersize=2, linestyle='-', label=f'Error')
        ax2 = axs.twinx()
        ax2.plot(time_list, error_dot, color=colors[(idx+1) % len(colors)], marker=markers[(idx+1) % len(markers)], markersize=2, linestyle='--', label=f'Error Dot')

        
        plt.xlabel('Time (s)')
        if "angular" in filename:
            plt.title(f'Heading Error and Error Dot over Time')
            axs.set_ylabel('Error(rad)')
            ax2.set_ylabel('Error Dot (m/s)')
        else:
            plt.title(f'Position Error and Error Dot over Time')
            axs.set_ylabel('Error(m)')
            ax2.set_ylabel('Error Dot (m/s)')
        axs.legend(loc='upper left')
        ax2.legend(loc='upper right')
        plt.grid(True)
        plt.savefig(filename[:-4]+"_e_edot_t_.svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
        # plt.show()


def plot_x_y_th_t(filenames):
    """
    Plots x, y, theta over time (t) for multiple files.

    Args:
        filenames (list): List of CSV file paths (e.g., robot_pose.csv).
    """
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    markers = ['o', 's', '^', 'D', 'v']
    

    for idx, filename in enumerate(filenames):
        fig, axes = plt.subplots(1, 1, figsize=(8, 6))
        
        headers, values = FileReader(filename).read_file()
        time_list = [val[-1] - values[0][-1] for val in values]
        if time_list[-1] > 1e10:
            time_list = [val / 1e9 for val in time_list]
        x = [val[0] for val in values]
        y = [val[1] for val in values]
        theta = [val[2] for val in values]

        label_base = filename.split('/')[-1].replace('_pose.csv', '').replace('robot', 'Robot')
        axes.plot(time_list, x, color=colors[0], marker=markers[0], markersize=2, linestyle='-', label=f'X')
        axes.plot(time_list, y, color=colors[1], marker=markers[1], markersize=2, linestyle='-', label=f'Y')

        axes.set_title('X, Y, Theta over Time')
        axes.set_ylabel('X/Y Displacement (m)')
        axes.set_xlabel('Time (s)')
        axes.grid(True)

        ax2 = axes.twinx()
        ax2.set_ylabel('Theta Displacement (rad)')

        ax2.plot(time_list, theta, color=colors[2], marker=markers[2], markersize=2, linestyle='-', label=f'Theta')
        axes.legend(loc='upper left')
        ax2.legend(loc='upper right')
        plt.tight_layout()
        plt.savefig(filename[:-4]+"_x_y_th_t.svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
        # plt.show()


def plot_x_y(filenames):
    """
    Plots x vs y (state space) for multiple files.

    Args:
        filenames (list): List of CSV file paths (e.g., robot_pose.csv).
    """
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    markers = ['o', 's', '^', 'D', 'v']
    #titles = ["Parabolla", "Sigmoid"] #use if trajectory mapping
    

    for idx, filename in enumerate(filenames):
        plt.figure(figsize=(8, 6))
        headers, values = FileReader(filename).read_file()
        x = [val[0] for val in values]
        y = [val[1] for val in values]

        label_base = filename.split('/')[-1].replace('_pose.csv', '').replace('robot', 'Robot')
        plt.plot(x, y, color=colors[idx % len(colors)], marker=markers[idx % len(markers)], markersize=2, linestyle='-', label=f'{label_base}')

        plt.title(f'Trajectory X vs Y (State Space)')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid(True)
        plt.savefig(filename[:-4]+"_x_y.svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
        # plt.show()


def plot_e_edot(filenames):
    """
    Plots error vs error_dot for multiple files.

    Args:
        filenames (list): List of CSV file paths (e.g., angular_errors.csv or linear_errors.csv).
    """
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    markers = ['o', 's', '^', 'D', 'v']
    

    for idx, filename in enumerate(filenames):
        plt.figure(figsize=(8, 6))
        headers, values = FileReader(filename).read_file()
        error = [val[0] for val in values]
        error_dot = [val[1] for val in values]

        label_base = filename.split('/')[-1].replace('_errors.csv', '').replace('angular', 'Angular').replace('linear', 'Linear')
        plt.plot(error, error_dot, color=colors[idx % len(colors)], marker=markers[idx % len(markers)], markersize=2, linestyle='-', label=f'{label_base}')

        if "angular" in filename:
            plt.title('Heading Error vs Error Dot')
            plt.xlabel('Error (rad)')
            plt.ylabel('Error Dot (rad/s)')
        else:
            plt.title('Position Error vs Error Dot')
            plt.xlabel('Error (m)')
            plt.ylabel('Error Dot (m/s)')
        
        plt.grid(True)
        plt.savefig(filename[:-4]+"_e_edot.svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
        # plt.show()


def plot_errors(filename):
    """
    Plots error data from a given CSV file.

    Reads the file using FileReader, extracts headers and values,
    normalizes timestamps, and creates subplots for state space and
    individual state errors.

    Args:
        filename (str): Path to the CSV file containing error data.
    """
    # Read headers and values from the file
    headers, values = FileReader(filename).read_file()

    # Initialize list for normalized timestamps
    time_list = []

    # Get the first timestamp as reference
    first_stamp = values[0][-1]

    # Normalize timestamps by subtracting the first stamp
    for val in values:
        time_list.append(val[-1] - first_stamp)

    # Create a figure with two subplots
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Plot state space (x vs y)
    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[0].set_title("state space")
    axes[0].grid()

    # Plot each individual state over time
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i] + " linear")

    axes[1].legend()
    axes[1].grid()

    # Display the plot
    plt.show()


import argparse

if __name__ == "__main__":
    # Set up argument parser for command-line usage
    parser = argparse.ArgumentParser(description='Plot error data from CSV files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of CSV files to process')
    parser.add_argument('--plot_type', choices=['e_edot_t', 'x_y_th_t', 'x_y', 'e_edot'], required=True, help='Type of plot to generate')

    # Parse arguments
    args = parser.parse_args()

    # Print the files being plotted
    print(f"Plotting {args.plot_type} for files: {args.files}")

    # Call the appropriate plot function
    if args.plot_type == 'e_edot_t':
        plot_e_edot_t(args.files)
    elif args.plot_type == 'x_y_th_t':
        plot_x_y_th_t(args.files)
    elif args.plot_type == 'x_y':
        plot_x_y(args.files)
    elif args.plot_type == 'e_edot':
        plot_e_edot(args.files)



