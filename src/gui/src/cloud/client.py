import socket
import psutil
import time
import requests

def has_internet_connection(host="8.8.8.8", port=53, timeout=1):
    try:
        socket.setdefaulttimeout(timeout)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((host, port))
        return True
    except socket.error:
        return False

def get_active_ipv4_interface():
    for interface, addrs in psutil.net_if_addrs().items():
        for addr in addrs:
            if addr.family == socket.AF_INET and not addr.address.startswith("127."):
                stats = psutil.net_if_stats().get(interface)
                if stats and stats.isup:
                    return interface, addr.address
    return None, None

def wait_for_active_connection(poll_interval=2):
    while True:
        if has_internet_connection():
            iface, ip = get_active_ipv4_interface()
            if iface and ip:
                return iface, ip
        time.sleep(poll_interval)

def post_ip_to_worker():
    iface, ip = wait_for_active_connection()
    requests.post("https://ip-broadcaster-worker.yu-qing-liu.workers.dev", data=ip)

if __name__ == "__main__":
    post_ip_to_worker()
