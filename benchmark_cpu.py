import argparse
from multiprocessing import Pipe, Process
import time

import cv2
import numpy as np
import psutil
from imutils.video import VideoStream

PROCESS_READY = "ready"
PROCESS_BUSY = "busy"
KERNEL_SIZE = (10, 10)

# Arguments definition
parser = argparse.ArgumentParser()
parser.add_argument(
    "-b",
    "--blur_count",
    type=int,
    default=1,
    help="number of cv.medianBlur() to create in each process",
)
parser.add_argument(
    "-p",
    "--process_count",
    type=int,
    default=3,
    help="number of cv pipe processes to spawn, not counting the video buffer",
)
parser.add_argument(
    "-m",
    "--monitor_count",
    type=int,
    default=11,
    help="the number of times to take a sample of monitoring CPU usage",
)
parser.add_argument(
    "-i",
    "--monitor_interval",
    type=float,
    default=0.5,
    help="the interval of taking monitoring samples (in seconds)",
)
parser.add_argument(
    "-t",
    "--monitor_total_cpu",
    action="store_false",
    default=True,
    help="call argument to show overall core usage (not per cpu)",
)
parser.add_argument(
    "-f",
    "--frame_hide",
    action="store_true",
    default=False,
    help="call argument to hide the frame (skip cv2.imshow)",
)
args = parser.parse_args()


# Function to read video frames from a camera stream
def video_buffer(conn_out):
    vs = VideoStream(src=0).start()  # Open the camera
    time.sleep(2.0)  # allow the camera sensor to warm up

    try:
        while True:
            frame = vs.read()

            detect_status = conn_out.recv()
            if detect_status == PROCESS_READY:
                conn_out.send(frame)
            if detect_status == PROCESS_BUSY:
                pass

    except KeyboardInterrupt:
        pass


def cv_func(conn_in, conn_out):
    """Simulate a computer vision pipeline by blurring the frame a number of times.

    :param numpy.ndarray conn_in: the connection to receive the frame
    :param numpy.ndarray conn_out: the connection to send the frame
    """
    try:
        while True:
            conn_in.send(PROCESS_READY)
            frame = conn_in.recv()
            conn_in.send(PROCESS_BUSY)

            accept_status = conn_out.recv()
            if accept_status == PROCESS_READY:
                for _ in range(args.blur_count):
                    frame = cv2.medianBlur(frame, 19)
                conn_out.send(frame)
            if accept_status == PROCESS_BUSY:
                pass

    except KeyboardInterrupt:
        pass


def monitor_cpu_usage(pid_list):
    print("Printing CPU usage for each process, and usage of each CPU, all in %")
    print("loading...")
    time.sleep(3.0)  # wait for the other processes to be spawned

    psu_list = [psutil.Process(pid) for pid in pid_list]

    for count in range(args.monitor_count):
        # monitor the cpu usage of each process
        p_list = [
            psu.cpu_percent(interval=None) for psu in psu_list
        ]  # set to None so it is non-blocking

        # monitor the overall cpu usage
        cpu = psutil.cpu_percent(
            interval=args.monitor_interval, percpu=args.monitor_total_cpu
        )

        # Print the results
        print(f"----- Count {count} -----")
        print(f"process: {p_list}")
        print(f"cpu: {cpu}")

    if args.frame_hide:
        print("Monitoring finished. To exit, do a KeyboardInterrupt (ctrl + c).")
    else:
        print("Monitoring finished. To exit, press 'Esc' key.")


def run_multi_pipe():
    # add 1 more pipe for the video stream process
    cv_pipes = [Pipe() for i in range(args.process_count + 1)]

    vs_process = Process(target=video_buffer, args=(cv_pipes[0][1],))

    cv_processes = [
        Process(
            target=cv_func,
            args=(cv_pipes[i][0], cv_pipes[i + 1][1]),
        )
        for i in range(args.process_count)
    ]

    # the list for storing PIDs for monitoring
    pid_list = []

    vs_process.start()
    pid_list.append(vs_process.pid)

    for cv_process in cv_processes:
        cv_process.start()
        pid_list.append(cv_process.pid)

    # start the monitoring in a separate process
    monitor_process = Process(target=monitor_cpu_usage, args=(pid_list,))
    monitor_process.start()

    try:
        # display frames
        while True:
            # receive frames from the last connection of the whole pipeline
            last_conn = cv_pipes[-1][0]
            last_conn.send(PROCESS_READY)
            frame = last_conn.recv()
            last_conn.send(PROCESS_BUSY)

            if args.frame_hide is not True:
                cv2.imshow("Benchmarking", frame)

            ch = cv2.waitKey(1)
            if ch == 27:  # Press escape key to exit
                break

    except KeyboardInterrupt:
        pass

    # cleanup
    monitor_process.terminate()
    vs_process.terminate()
    for cv_process in cv_processes:
        cv_process.terminate()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_multi_pipe()
