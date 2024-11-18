import matplotlib.pyplot as plt
from utilities import FileReader

def plot_errors(filename):
    headers, values = FileReader(filename).read_file()

    time_list = []
    first_stamp = values[0][-1]
    for val in values:
        time_list.append(val[-1] - first_stamp)

    # code to overlay the odom and the predicted (kalman filter) x/y posns
    kf_x_index = headers.index("kf_x")
    kf_y_index = headers.index("kf_y")
    odom_x_index = headers.index("odom_x")
    odom_y_index = headers.index("odom_y")

    fig, axes = plt.subplots(2, 1, figsize=(14, 6))

    plt.subplots_adjust(hspace=0.4)  # Increase vertical space between subplots

    axes[0].plot([lin[kf_x_index] for lin in values], [lin[kf_y_index] for lin in values], label="KF (x, y)")
    axes[0].plot([lin[odom_x_index] for lin in values], [lin[odom_y_index] for lin in values], label="Odom (x, y)", linestyle="--")
    axes[0].set_title("State Space Plot (KF vs Odom)")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend()
    axes[0].grid()

    axes[1].set_title("Each Individual State")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i])

    axes[1].set_xlabel("time [seconds]")
    axes[1].set_ylabel("various units")
    axes[1].legend()
    axes[1].grid()

    plt.show()

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    args = parser.parse_args()
    print("Plotting the files", args.files)

    filenames = args.files
    for filename in filenames:
        plot_errors(filename)
