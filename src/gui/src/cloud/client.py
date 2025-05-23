import requests

def get_public_ip():
    return requests.get("https://api.ipify.org").text

def post_ip_to_worker():
    ip = get_public_ip()
    requests.post("https://ip-broadcaster-worker.yu-qing-liu.workers.dev", data=ip)

if __name__ == "__main__":
    post_ip_to_worker()
