import socket
from typing import Dict, List, Sequence, Tuple


def create_udp_socket(
    bind_host: str = "",
    bind_port: int = 0,
    nonblocking: bool = True,
) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if bind_host or bind_port:
        sock.bind((bind_host, int(bind_port)))
    sock.setblocking(not nonblocking)
    return sock


def recv_udp_messages(sock: socket.socket, max_bytes: int = 65535) -> List[Tuple[bytes, Tuple[str, int]]]:
    messages: List[Tuple[bytes, Tuple[str, int]]] = []
    while True:
        try:
            payload, addr = sock.recvfrom(max_bytes)
        except BlockingIOError:
            break
        messages.append((payload, addr))
    return messages


def parse_robot_targets(items: Sequence[str]) -> Dict[str, Tuple[str, int]]:
    targets: Dict[str, Tuple[str, int]] = {}
    for raw_item in items:
        item = str(raw_item).strip()
        if not item or "=" not in item:
            continue
        robot_name, endpoint = item.split("=", 1)
        robot_name = robot_name.strip()
        endpoint = endpoint.strip()
        if not robot_name or ":" not in endpoint:
            continue
        host, port_text = endpoint.rsplit(":", 1)
        host = host.strip()
        try:
            port = int(port_text.strip())
        except ValueError:
            continue
        if not host or port <= 0:
            continue
        targets[robot_name] = (host, port)
    return targets


def parse_robot_targets_csv(raw: str) -> Dict[str, Tuple[str, int]]:
    text = str(raw).strip()
    if not text:
        return {}
    parts = [item.strip() for item in text.split(",")]
    return parse_robot_targets(parts)
