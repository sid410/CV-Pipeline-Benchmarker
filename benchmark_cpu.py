import argparse
import multiprocessing
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
    help="number of processes to spawn, with 1 meaning only the video buffer gets created",
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


def cv_pipe(conn_in, conn_out):
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


def monitor_cpu_usage(process_dictionary):
    print("Printing CPU usage for each process, and usage of each CPU, all in %")
    print("loading...")
    time.sleep(3.0)  # wait for the other processes to be spawned

    # a list to store the psutil processes
    psu_list = []

    for process in process_dictionary:
        p = process_dictionary[process]
        psu_list.append(psutil.Process(p.pid))

    for count in range(args.monitor_count):
        # monitor the cpu usage of each process
        u_list = []
        for psu in psu_list:
            cpu_usage = psu.cpu_percent(
                interval=None
            )  # set to None so it is non-blocking
            u_list.append(cpu_usage)

        # monitor the overall cpu usage
        cpu = psutil.cpu_percent(
            interval=args.monitor_interval, percpu=args.monitor_total_cpu
        )

        # Print the results
        print(f"-----Count {count}-----")
        print(f"process: {u_list}")
        print(f"cpu: {cpu}")

    if args.frame_hide:
        print("Monitoring finished. To exit, do a KeyboardInterrupt (ctrl + c).")
    else:
        print("Monitoring finished. To exit, press 'Esc' key.")


# create and get pipe names. In for loop starting index 0, name is pipe0_1
def get_pipe_name(p_index: int) -> str:
    pipe_name = f"pipe{p_index}_{p_index+1}"
    return str(pipe_name)


# create and get process names. In for loop starting index 0, name is process0
def get_process_name(p_index: int) -> str:
    process_name = f"process{p_index}"
    return str(process_name)


def main():
    # Check because we need at least 1 process to be spawned
    if args.process_count < 1:
        print(
            f"You tried to spawn {args.process_count} processes.\
            \nPlease input a number between 1 and total logical cores minus 2."
        )
        return

    # create dictionaries for reference
    pipe_dict = {}
    process_dict = {}

    # create pipes
    for pipe_index in range(args.process_count):
        pipe_dict[get_pipe_name(pipe_index)] = multiprocessing.Pipe()

    # create processes
    for process_index in range(args.process_count):
        # spawn a get cam frames from start then apply cv methods for the succeeding
        if process_index == 0:
            process_dict[get_process_name(process_index)] = multiprocessing.Process(
                target=video_buffer, args=(pipe_dict[get_pipe_name(process_index)][1],)
            )
        else:
            process_dict[get_process_name(process_index)] = multiprocessing.Process(
                target=cv_pipe,
                args=(
                    pipe_dict[get_pipe_name(process_index - 1)][0],
                    pipe_dict[get_pipe_name(process_index)][1],
                ),
            )

    # start the processes
    for process in process_dict:
        p = process_dict[process]
        p.start()

    # start the monitoring in a separate process
    monitor_process = multiprocessing.Process(
        target=monitor_cpu_usage, args=(process_dict,)
    )
    monitor_process.start()

    try:
        # display frames
        while True:
            # receive frames from the last connection of the whole pipeline
            last_conn = pipe_dict[get_pipe_name(args.process_count - 1)][0]
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
    for process in process_dict:
        p = process_dict[process]
        p.terminate()

    monitor_process.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
