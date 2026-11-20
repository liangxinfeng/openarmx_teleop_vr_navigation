"""Linux ownership of controller launch resources, shared by the desktop and ROS launches."""
import atexit
import ctypes
import fcntl
from contextlib import ExitStack
from types import SimpleNamespace
import os
from pathlib import Path
import secrets
import select
import signal
import subprocess
import sys
import time


_RESOURCES = {'navigation': ('chassis',), 'slam': ('chassis',), 'joint': ('joints',),
              'grasp': ('joints',), 'vr_arm': ('joints',), 'vr_stack': ('chassis', 'joints')}
_TAG = 'AUTOLIFE_CONTROLLER_GENERATION_'


# Older launchers have no lease. Check their controller executables before
# granting ownership during a rolling update; never signal an untagged process.
_LEGACY = {'chassis': ('/nav2_map_server/map_server', '/nav2_controller/controller_server',
                        '/cartographer_ros/cartographer_occupancy_grid_node',
                        '/control_center_navigation/cmd_vel_arbiter_node.py'),
           'joints': ('/control_center_joint_control/joint_bridge',)}


def _legacy_owner(resource, domain):
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            if int(proc.name) == os.getpid():
                continue
            args = (proc / 'cmdline').read_bytes().split(b'\0')[:4]
            actor = resource == 'joints' and (args[1:3] == [b'-m', b'autolife_shelf_grasp_306.app'] or
                    any(b'openarmx_306_v5_arm_controller' in a for a in args[:2]))
            flow = any(line.rstrip('/').endswith('/flow-service.service') for line in
                       (proc / 'cgroup').read_text().splitlines())
            if not actor and not flow and not any(os.fsdecode(arg).endswith(_LEGACY[resource]) for arg in args[:2]):
                continue
            env = dict(item.split(b'=', 1) for item in (proc / 'environ').read_bytes().split(b'\0') if b'=' in item)
            if env.get(b'ROS_DOMAIN_ID', b'0') == domain.encode():
                return proc.name
        except (FileNotFoundError, ProcessLookupError):
            continue
    return None


def _libc_call(name, *args):
    libc = ctypes.CDLL(None, use_errno=True)
    result = getattr(libc, name)(*args)
    if result < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    return result


def open_pid(pid):
    if hasattr(os, 'pidfd_open'):
        return os.pidfd_open(pid)
    return _libc_call('pidfd_open', pid, 0)


def _send(fd, sig):
    if hasattr(signal, 'pidfd_send_signal'):
        signal.pidfd_send_signal(fd, sig)
    else:
        _libc_call('pidfd_send_signal', fd, int(sig), None, 0)


def _members(token):
    """Return pidfds pinned to processes that inherited this exact launch token."""
    keys = {f'{_TAG}{resource}{layer}={token}'.encode() for resource in ('chassis', 'joints')
            for layer in ('', '_backend')}
    handles = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        fd = None
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            fd = open_pid(int(proc.name))
            if not keys.isdisjoint((proc / 'environ').read_bytes().split(b'\0')):
                handles.append(fd)
                fd = None
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
        finally:
            if fd is not None:
                os.close(fd)
    return handles


def _drain(token):
    start = time.monotonic()
    while True:
        members = _members(token)
        if not members:
            return
        sig = signal.SIGTERM if time.monotonic() - start < 2 else signal.SIGKILL
        for fd in members:
            try:
                _send(fd, sig)
            except ProcessLookupError:
                pass
            finally:
                os.close(fd)
        time.sleep(.1)


def _watch(parent_fd, lock_fds, token, module, pid):
    poll = select.poll()
    poll.register(parent_fd, select.POLLIN)
    poll.poll()
    _drain(token)
    if module in ('navigation', 'slam', 'joint'):
        from control_center_shell.preflight import forget_process
        forget_process(module, pid)
    for fd in lock_fds:
        os.ftruncate(fd, 0)


def claim(context, module):
    directory = Path.home() / '.local/state/autolife-control-center/leases'
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    token = secrets.token_hex(16)
    locks = {}
    with ExitStack() as stack:
        for resource in _RESOURCES[module]:
            # The VR composite reserves both resources before its service handoff.
            # Its included launches each acquire a separate single-backend lock.
            layers = ('',) if module == 'vr_stack' else ('', '_backend')
            for layer in layers:
                key = resource + layer
                lock = stack.enter_context((directory / (key + '.lock')).open('a+'))
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    lock.seek(0)
                    owner = lock.read().strip()
                    if not layer and owner and owner == context.environment.get(_TAG + key):
                        continue
                    raise RuntimeError(f'{resource} 正由其他功能包使用或正在清理') from None
                locks[key] = lock
                lock.seek(0)
                prior = lock.read().strip()
                if prior:
                    if len(prior) != 32 or any(c not in '0123456789abcdef' for c in prior):
                        raise RuntimeError(f'{resource} 上次占用记录无效')
                    _drain(prior)
                if not layer:
                    legacy = _legacy_owner(resource, context.environment.get('ROS_DOMAIN_ID', '0'))
                    if legacy:
                        raise RuntimeError(f'{resource} 已有控制进程 PID {legacy}，请由原应用释放')
        if not locks:
            return
        # Publish ownership only after every non-blocking acquisition succeeded.
        for lock in locks.values():
            lock.seek(0)
            lock.truncate()
            lock.write(token)
            lock.flush()
        parent = open_pid(os.getpid())
        fds = tuple(lock.fileno() for lock in locks.values())
        try:
            env = {k:v for k,v in os.environ.items() if not k.startswith(_TAG)}
            subprocess.Popen([sys.executable, __file__, 'watch', str(parent), token,
                              module, str(os.getpid()), *map(str,fds)],
                             pass_fds=(parent,*fds), env=env, start_new_session=True,
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        finally:
            os.close(parent)
        for key, lock in locks.items():
            atexit.register(os.close, os.dup(lock.fileno()))
            context.environment[_TAG + key] = token


def controller_lease(module):
    from launch.actions import OpaqueFunction
    return OpaqueFunction(function=claim, args=[module])


if __name__ == '__main__':
    if sys.argv[1] == 'watch':
        args = sys.argv[4:]
        module, pid = '', 0
        if not args[0].isdigit():
            module, pid = args[0], int(args[1])
            args = args[2:]
        _watch(int(sys.argv[2]), tuple(map(int, args)), sys.argv[3], module, pid)
    elif sys.argv[1] == 'child':
        # SIGKILL is used only for unexpected loss of the owning GUI. The
        # independent lease watcher then drains this launch's tagged workers.
        _libc_call('prctl', 1, int(signal.SIGKILL), 0, 0, 0)  # PR_SET_PDEATHSIG
        if os.getppid() != int(sys.argv[2]):
            raise SystemExit('控制窗口已经退出')
        os.setsid()
        os.execvp(sys.argv[3], sys.argv[3:])
    elif sys.argv[1] == 'run':
        environment = dict(os.environ)
        claim(SimpleNamespace(environment=environment), sys.argv[2])
        os.execvpe(sys.argv[3], sys.argv[3:], environment)
