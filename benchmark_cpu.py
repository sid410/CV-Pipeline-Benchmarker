import argparse
import time
from collections import deque
from multiprocessing import Pipe, Process, Event
from multiprocessing.pool import ThreadPool

import cv2
import imutils
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
    "-t",
    "--thread_count",
    type=int,
    default=0,
    help="number of threads to make per process",
)
parser.add_argument(
    "-mc",
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
    "-tc",
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


class FPS:
    def __init__(self):
        self.start_time = 0
        self.frames = 0

    def start(self):
        self.start_time = time.perf_counter()
        self.frames = 0

    def update(self):
        self.frames += 1

    def stop(self):
        elapsed_time = time.perf_counter() - self.start_time
        fps = self.frames / elapsed_time
        return fps


# Function to read video frames from a camera stream
def video_buffer(conn_out):
    vs = VideoStream(src=0).start()  # Open the camera
    time.sleep(2.0)  # allow the camera sensor to warm up

    try:
        while True:
            frame = vs.read()

            status = conn_out.recv()
            if status == PROCESS_READY:
                frame = imutils.resize(frame, width=1280, height=720)
                conn_out.send(frame)

            else:
                pass

    except KeyboardInterrupt:
        pass


def blur_loop(frame, times):
    for _ in range(times):
        frame = cv2.medianBlur(frame, 19)
    return frame


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

            status = conn_out.recv()
            if status == PROCESS_READY:
                frame = blur_loop(frame, args.blur_count)
                conn_out.send(frame)

    except KeyboardInterrupt:
        pass


def cv_func_threaded(conn_in, conn_out, thread_num):
    """Simulate a computer vision pipeline by blurring the frame a number of times.

    :param numpy.ndarray conn_in: the connection to receive the frame
    :param numpy.ndarray conn_out: the connection to send the frame
    """

    pool = ThreadPool(processes=thread_num)
    pending_task = deque()

    try:
        while True:
            # Consume the queue.
            while len(pending_task) > 0 and pending_task[0].ready():
                frame = pending_task.popleft().get()
                status = conn_out.recv()
                if status == PROCESS_READY:
                    conn_out.send(frame)

            # Populate the queue.
            if len(pending_task) < thread_num:
                conn_in.send(PROCESS_READY)
                frame = conn_in.recv()
                conn_in.send(PROCESS_BUSY)
                task = pool.apply_async(blur_loop, (frame.copy(), args.blur_count))
                pending_task.append(task)

    except KeyboardInterrupt:
        pass


def monitor_cpu_usage(pid_list, event):
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

    event.set()

    if args.frame_hide:
        print("Monitoring finished. To exit, do a KeyboardInterrupt (ctrl + c).")
    else:
        print("Monitoring finished. To exit, press 'Esc' key.")


def run_multi_pipe():
    stop_event = Event()

    # add 1 more pipe for the video stream process
    cv_pipes = [Pipe() for i in range(args.process_count + 1)]

    vs_process = Process(target=video_buffer, args=(cv_pipes[0][1],))

    if args.thread_count < 1:
        cv_processes = [
            Process(
                target=cv_func,
                args=(cv_pipes[i][0], cv_pipes[i + 1][1]),
            )
            for i in range(args.process_count)
        ]
    else:
        cv_processes = [
            Process(
                target=cv_func_threaded,
                args=(cv_pipes[i][0], cv_pipes[i + 1][1], args.thread_count),
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
    monitor_process = Process(target=monitor_cpu_usage, args=(pid_list, stop_event))
    monitor_process.start()

    fps_measure = FPS()
    fps_measure.start()

    try:
        # display frames
        while True:
            # receive frames from the last connection of the whole pipeline
            last_conn = cv_pipes[-1][0]
            last_conn.send(PROCESS_READY)
            frame = last_conn.recv()
            last_conn.send(PROCESS_BUSY)

            fps_measure.update()

            if args.frame_hide is not True:
                cv2.imshow("Benchmarking", frame)

            if stop_event.is_set():  # stop when monitoring is finished
                break

            ch = cv2.waitKey(1)
            if ch == 27:  # Press escape key to exit
                break

    except KeyboardInterrupt:
        pass

    average_fps = fps_measure.stop()
    print(f"average fps: {average_fps}")

    # cleanup
    monitor_process.terminate()
    vs_process.terminate()
    for cv_process in cv_processes:
        cv_process.terminate()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_multi_pipe()
