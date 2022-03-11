import pathlib
import subprocess
import time
import sys

signaling_server_path = pathlib.Path(__file__).parent.parent.parent.parent.parent.joinpath('signaling-server') \
    .joinpath('opentera-signaling-server').absolute()
ice_server_path = pathlib.Path(__file__).parent.joinpath('resources').joinpath('iceServers.json').absolute()


class SignalingServerRunner:
    def __init__(self):
        # Use same interpreter
        self._process = subprocess.Popen([sys.executable, signaling_server_path, '--port', '8080', '--password', 'abc',
                                          '--ice_servers', ice_server_path])
        time.sleep(3)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self):
        self._process.kill()
        self._process.wait()
