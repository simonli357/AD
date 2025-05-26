import subprocess
import psutil
import socket
import time
import requests
import sys

def has_internet_connection_ping(host="8.8.8.8", count=1, timeout=2):
    try:
        result = subprocess.run(
            ["ping", "-c", str(count), "-W", str(timeout), host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return result.returncode == 0
    except Exception:
        return False

def get_ipv4_from_interface(interface="wlan0"):
    addrs = psutil.net_if_addrs().get(interface, [])
    for addr in addrs:
        if addr.family == socket.AF_INET and not addr.address.startswith("127."):
            return addr.address
    return None

def wait_for_connection_and_ip(interface="wlan0", poll_interval=2):
    print("Waiting for internet connection and valid IP on interface:", interface)
    while True:
        connected = has_internet_connection_ping()
        ip = get_ipv4_from_interface(interface)
        print(f"[Debug] Ping: {connected}, IP: {ip}")
        if connected and ip:
            return ip
        time.sleep(poll_interval)

def post_ip_to_worker(ip):
    try:
        response = requests.post("https://ip-broadcaster-worker.yu-qing-liu.workers.dev", data=ip)
        response.raise_for_status()
        print(f"Posted IP {ip} successfully.")
    except Exception as e:
        print(f"Failed to post IP: {e}", file=sys.stderr)

if __name__ == "__main__":
    ip = wait_for_connection_and_ip("wlan0")
    post_ip_to_worker(ip)

