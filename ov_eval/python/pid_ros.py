#!/usr/bin/env python3

# OpenVINS: An Open Platform for Visual-Inertial Research
# Copyright (C) 2019 Patrick Geneva
# Copyright (C) 2019 OpenVINS Contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys
import time
import psutil
import rclpy
from rclpy.node import Node

def get_process_ros2(node_name, logger, doprint=False):
    """
    Finds a ROS 2 node process by its name using several strategies.
    """
    # Use the base name for searching (e.g., 'run_subscribe_msckf' from '/ov_msckf/run_subscribe_msckf')
    base_node_name = node_name.split('/')[-1]
    
    for p in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmdline = p.info['cmdline']
            if not cmdline:
                continue

            # --- Strategy 1: Check for executable path match ---
            # This works for nodes started with `ros2 run <pkg> <exec>`.
            # e.g., cmdline[0] might be '/.../install/pkg/lib/pkg/executable_name'
            executable_path = cmdline[0]
            if executable_path.split('/')[-1] == base_node_name:
                if doprint:
                    logger.info(f"FOUND (executable name match): Adding new node monitor {node_name} (pid {p.info['pid']})")
                return p

            # --- Strategy 2: Check for ROS 2 name remapping argument ---
            # This works if the node is launched with a name remap.
            # e.g., `... --ros-args -r __node:=my_node_name`
            search_str_remap = f"__node:={base_node_name}"
            if any(search_str_remap in arg for arg in cmdline):
                if doprint:
                    logger.info(f"FOUND (remap argument match): Adding new node monitor {node_name} (pid {p.info['pid']})")
                return p
            
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    
    logger.warn(f"Could not find process for node '{node_name}' (tried searching for base name '{base_node_name}')")
    return None


def main(args=None):
    rclpy.init(args=args)
    node = Node("pid_ros")
    logger = node.get_logger()

    # Declare and get parameters
    node.declare_parameter('nodes', '')
    node.declare_parameter('output', '')
    node_csv = node.get_parameter('nodes').get_parameter_value().string_value
    save_path = node.get_parameter('output').get_parameter_value().string_value

    if not node_csv or not save_path:
        logger.error("Please specify the nodes and output file for this logger.")
        logger.error("ros2 run ov_eval pid_ros.py --ros-args -p nodes:=<comma,separated,node,names> -p output:=<file.txt>")
        sys.exit(-1)
        
    node_list = node_csv.split(',')

    # Debug print to console
    logger.info(f"Processes: {node_csv} ({len(node_list)} in total)")
    logger.info(f"Save path: {save_path}")

    # ===================================================================
    # ===================================================================

    # Make sure the directory is made
    if not os.path.exists(os.path.dirname(save_path)):
        try:
            os.makedirs(os.path.dirname(save_path))
        except:
            logger.error("Unable to create the save path!")
            sys.exit(-1)

    # Open the file we will write the stats into
    file = open(save_path, "w")

    # Write header to file
    header = "# timestamp(s) summed_cpu_perc summed_mem_perc summed_threads"
    for n in node_list:
        get_process_ros2(n, logger, True)  # nice debug print!
        header += f" {n}_cpu_perc {n}_mem_perc {n}_threads"
    header += "\n"
    file.write(header)

    # ===================================================================
    # ===================================================================

    # Now let's loop and get the stats for these processes
    rate = node.create_rate(1) # 1 Hz
    
    ps_list = [None] * len(node_list)

    try:
        while rclpy.ok():
            # Get the pid processes for this object
            for i, n in enumerate(node_list):
                if ps_list[i] is None or not ps_list[i].is_running():
                    ps_list[i] = get_process_ros2(n, logger, False)
            
            # This is to initialize the cpu_percent measurement
            for p in ps_list:
                if p:
                    try:
                        p.cpu_percent(interval=None)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
            
            # wait one second so we can collect data
            rate.sleep()

            # Loop through and get our measurement readings
            perc_cpu = []
            perc_mem = []
            threads = []
            for i in range(len(node_list)):
                try:
                    # Get readings
                    p_cpu = ps_list[i].cpu_percent(interval=None)
                    p_mem = ps_list[i].memory_percent()
                    p_threads = ps_list[i].num_threads()
                    # Append to our list
                    perc_cpu.append(p_cpu)
                    perc_mem.append(p_mem)
                    threads.append(p_threads)
                except (AttributeError, psutil.NoSuchProcess, psutil.AccessDenied):
                    # Record just zeros if we do not have this value
                    perc_cpu.append(0)
                    perc_mem.append(0)
                    threads.append(0)

            # Print what the total summed value is
            logger.info(f"cpu% = {sum(perc_cpu):.3f} | mem% = {sum(perc_mem):.3f} | threads = {sum(threads)}")

            # Save the current stats to file!
            data = f"{time.time():.8f} {sum(perc_cpu):.3f} {sum(perc_mem):.3f} {sum(threads)}"
            for i in range(len(node_list)):
                data += f" {perc_cpu[i]:.3f} {perc_mem[i]:.3f} {threads[i]}"
            data += "\n"
            file.write(data)
            file.flush()
    except KeyboardInterrupt:
        logger.info("Shutting down pid_ros.")
    finally:
        # Finally close the file and shutdown!
        file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
