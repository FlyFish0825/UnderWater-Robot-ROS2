"""Offline adapter integration: private localhost DDS, fake status/services, no UART.

Run after sourcing the isolated install: python3 this_file /path/to/remote_manual_node
Never launches gateway/serial nodes or publishes real-domain motor commands.
"""
import os
import socket
import subprocess
import sys
import time

os.environ["ROS_DOMAIN_ID"] = "179"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool
from underwater_interfaces.msg import AllocationStatus, GatewayStatus
from underwater_interfaces.srv import ValidateManualLease

TOKEN = "0123456789abcdef0123456789abcdef"
rclpy.init()
namespace = "/offline_manual_" + str(os.getpid())
node = rclpy.create_node("offline_gateway_fixture", namespace=namespace)
gateway_pub = node.create_publisher(GatewayStatus, "gateway/status", 1)
allocation_pub = node.create_publisher(AllocationStatus, "control/allocation", 1)
enabled = False
enable_calls = []
inputs = []
generation = 7
allow_lease = True
tx_enabled = False
hold_validation = False
pending_validation = []

async def validate(request, response):
    response.valid = allow_lease and request.token == TOKEN
    response.generation = generation if response.valid else 0
    response.detail = "offline fixture"
    if hold_validation:
        future = Future()
        pending_validation.append((future, response))
        return await future
    return response

def enable(request, response):
    global enabled
    enable_calls.append(request.data)
    enabled = request.data
    response.success = True
    return response

node.create_service(ValidateManualLease, "gateway/validate_manual_lease", validate,
    callback_group=ReentrantCallbackGroup())
node.create_service(SetBool, "control/enable", enable)
node.create_subscription(WrenchStamped, "control/manual_request", inputs.append, 1)

def publish_status():
    stamp = node.get_clock().now().to_msg()
    gateway = GatewayStatus()
    gateway.header.stamp = stamp
    gateway.connected = True
    gateway.tx_enabled = tx_enabled
    gateway.control_owner = "GUI_MANUAL"
    gateway.connection_id = 42
    gateway.heartbeat_age_ms = 0
    gateway.motor_session_id = 11 if enabled else 10
    gateway.motor_rearm_required = not enabled
    gateway_pub.publish(gateway)
    allocation = AllocationStatus()
    allocation.header.stamp = stamp
    allocation.armed = enabled
    allocation.session_id = 11 if enabled else 10
    allocation.solver_ok = True
    allocation.input_fresh = enabled
    allocation_pub.publish(allocation)

node.create_timer(0.02, publish_status)
probe = socket.socket()
probe.bind(("127.0.0.1", 0))
port = probe.getsockname()[1]
probe.close()
process = subprocess.Popen([sys.argv[1], "--ros-args", "-r", "__ns:=" + namespace,
    "-p", "bind_address:=127.0.0.1", "-p", "port:=" + str(port)],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
peer = None
wire = b""
closed = False

def pump(duration=0.02):
    global wire, closed
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        if peer is not None and not closed:
            try:
                data = peer.recv(4096)
                if data:
                    wire += data
                else:
                    closed = True
            except BlockingIOError:
                pass

def until(condition, timeout=4):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        pump()
    assert condition(), wire[-1200:]

def connect():
    global peer, wire, closed
    if peer is not None:
        peer.close()
    wire, closed = b"", False
    deadline = time.monotonic() + 4
    while True:
        try:
            peer = socket.create_connection(("127.0.0.1", port), timeout=0.2)
            peer.setblocking(False)
            break
        except OSError:
            if time.monotonic() > deadline:
                raise
            pump()
    until(lambda: b"HELLO ROV_MANUAL 2\n" in wire)

def send(line):
    peer.sendall((line + "\n").encode("ascii"))

try:
    connect()
    send("AUTH 1 " + "0" * 32)
    until(lambda: closed)
    assert not enable_calls and not inputs
    connect()
    send("AUTH 1 " + TOKEN)
    until(lambda: b"RESULT 1 1 0 " in wire)
    assert not enable_calls and not inputs
    # More than the original 3s deadline; periodic AUTH must not open a reservation gap.
    for sequence in range(2, 11):
        send("AUTH " + str(sequence) + " " + TOKEN)
        pump(0.5)
        assert not closed
    other = socket.create_connection(("127.0.0.1", port), timeout=1)
    other.setblocking(False)
    pump(0.1)
    assert other.recv(1) == b""
    other.close()
    send("ENABLE 20 " + TOKEN)
    until(lambda: b"RESULT 20 0 " in wire)
    assert not enable_calls and not inputs  # motor_tx=false never reaches enable service.
    tx_enabled = True  # Fake status only, private DDS domain; no physical gateway exists.
    pump(0.2)
    # A rejected ENABLE clears its reservation/proof. Mirror the GUI's idle AUTH
    # before testing renewal cancellation; changing status must not reauthorize it.
    send("AUTH 21 " + TOKEN)
    until(lambda: b"RESULT 21 1 0 " in wire)
    assert not enable_calls and not inputs
    hold_validation = True
    until(lambda: len(pending_validation) > 0) # Hold an existing idle renewal first.
    send("ENABLE 22 " + TOKEN)
    until(lambda: len(pending_validation) >= 2) # Explicit enable must start fresh validation.
    other = socket.create_connection(("127.0.0.1", port), timeout=1)
    other.setblocking(False)
    pump(0.05)
    assert other.recv(1) == b"" and not closed  # No reservation gap during async ENABLE.
    other.close()
    cancelled_future, cancelled_response = pending_validation.pop(0)
    cancelled_future.set_result(cancelled_response)
    pump(0.05)
    assert not enable_calls and not closed  # Old canceled reply cannot authorize ENABLE.
    hold_validation = False
    for future, response in pending_validation:
        future.set_result(response)
    pending_validation.clear()
    until(lambda: b"RESULT 22 1 11 " in wire)
    assert enable_calls == [True]
    send("INPUT 23 11 0 0 0 0 0 0")
    until(lambda: len(inputs) > 0)
    assert all(v.wrench.force.x == 0 for v in inputs)
    generation = 8  # Same token cannot authorize a changed gateway ownership generation.
    until(lambda: closed)
    until(lambda: False in enable_calls)
    assert not enabled
    print("PASS offline ROS adapter: AUTH, idle reservation, TX gate, zero ACK, generation loss")
finally:
    if peer is not None:
        peer.close()
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
