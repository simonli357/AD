import subprocess
import psutil

def get_ros_nodes():
    """ Retrieves a list of active ROS nodes. """
    try:
        print("Listing ROS nodes...")
        nodes = subprocess.check_output("rosnode list", shell=True).decode().splitlines()
    except subprocess.CalledProcessError as e:
        print("Failed to list ROS nodes:", e)
        return []
    return nodes

def get_node_pid(node_name):
    """ Retrieves the PID of a given ROS node by its name. """
    try:
        print(f"Getting info for node {node_name}...")
        node_info = subprocess.check_output(f"rosnode info {node_name}", shell=True).decode()
    except subprocess.CalledProcessError as e:
        print(f"Failed to get info for node {node_name}:", e)
        return None
    
    for line in node_info.splitlines():
        if "Pid" in line:
            return int(line.split()[-1])
    return None

def get_process_metrics(pid):
    """ Retrieves various usage statistics for a given PID. """
    try:
        print(f"Getting metrics for PID {pid}...")
        p = psutil.Process(pid)
        cpu_percent = p.cpu_percent(interval=1)  # measure CPU usage over 1 second
        mem_info = p.memory_info()
        num_threads = p.num_threads()

        return {
            'CPU': cpu_percent,
            'RSS': mem_info.rss / 1024 ** 2,  # Resident Set Size in MB
            'VMS': mem_info.vms / 1024 ** 2,  # Virtual Memory Size in MB
            'Threads': num_threads
        }
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return None

def print_formatted_table(nodes_data):
    """ Prints the nodes data formatted as a table. """
    header = f"{'Node':<30} {'PID':>10} {'CPU %':>10} {'RSS (MB)':>10} {'VMS (MB)':>15} {'Threads':>10}"
    print(header)
    print('-' * len(header))
    
    for data in nodes_data:
        if data['metrics']:
            print(f"{data['node_name']:<30} {data['pid']:>10} {data['metrics']['CPU']:>10.1f} {data['metrics']['RSS']:>10.2f} {data['metrics']['VMS']:>15.2f} {data['metrics']['Threads']:>10}")
        else:
            print(f"{data['node_name']:<30} {data['pid']:>10} {'N/A':>10} {'N/A':>10} {'N/A':>15} {'N/A':>10}")

def main():
    nodes = get_ros_nodes()
    nodes_data = []
    for node in nodes:
        pid = get_node_pid(node)
        if pid is None:
            continue

        metrics = get_process_metrics(pid)
        if len(node) > 15:
            node = node[:25] + "..."
        nodes_data.append({'node_name': node, 'pid': pid, 'metrics': metrics})
    
    print_formatted_table(nodes_data)

if __name__ == "__main__":
    main()
