import asyncio
import functools
import time

input_delimiter = b' '
output_delimiter = b'\r'


class Axis:
    def __init__(self, actual_position=0,
                 desired_position=0,
                 torque=0,
                 temperature=22,
                 status=None):

        if status is None:
            status = {}

        self.actual_position = int(actual_position)
        self.ext_encoder_position = int(actual_position)
        self.desired_position = int(desired_position)
        self.torque = 0
        self.temperature = 22
        self.status = {}
        self.brake_engaged = True
        self.closed_loop = False
        self.velocity = 0
        self.desired_velocity = 0
        self.desired_acceleration = 0

    def get_status(self, index):
        return self.status.get(index, 0)

    def set_open_loop(self):
        self.closed_loop = False
        self.brake_engaged = True
        # TODO: what does the controller respond here?
        return b''

    async def move(self):
        start_pos = self.actual_position
        desired_pos = self.desired_position
        delta_pos = desired_pos - start_pos

        velocity = self.velocity
        dt = delta_pos / velocity

        t0 = time.monotonic()
        elapsed = 0
        while elapsed < dt:
            if self.brake_engaged or not self.closed_loop:
                return
            self.actual_position = start_pos + (elapsed / dt) * delta_pos
            await asyncio.sleep(0.1)
            elapsed = time.monotonic() - t0

        self.ext_encoder_position = desired_pos
        self.actual_position = desired_pos


class SimState:
    addresses = {127 + c: c
                 for c in range(1, 34)}

    def __init__(self, positions=None,
                 frequency=8000,
                 firmware='440C'):
        if positions is None:
            positions = {}

        self.axes = {axis: Axis(actual_position=pos,
                                desired_position=pos)
                     for axis, pos in positions.items()
                     }
        self.variables = {}
        self.frequency = frequency
        self.firmware = firmware

    def handle_print(self, axis, command, param=None):
        'Read actual position'
        def _print(item):
            if item == 'TEMP':
                return str(self.axes[axis].temperature)
            elif item.startswith('"') and item.endswith('"'):
                return item.strip('"')
            elif item.startswith('#'):
                num = int(item.lstrip('#'))
                return chr(num)

        return ''.join(_print(item) for item in param.split(','))

    def handle_rctr(self, axis, command, param=None):
        'Read internal/external encoder position'
        if param is None or int(param) == 0:
            return str(self.axes[axis].actual_position)
        else:
            return str(self.axes[axis].ext_encoder_position)

    def handle_off(self, axis, command, param=None):
        'Set open loop'
        return self.axes[axis].set_open_loop()

    def handle_rpt(self, axis, command, param=None):
        'Read target position'
        return self.axes[axis].desired_position

    def handle_rpa(self, axis, command, param=None):
        'Read absolute actual position'
        return self.axes[axis].actual_position

    handle_rp = handle_rpa  # TODO?

    def handle_rt(self, axis, command, param=None):
        'Read torque'
        return self.axes[axis].torque

    def handle_rw(self, axis, command, param=0):
        'Read status word'
        return self.axes[axis].get_status(int(param))

    def handle_rsp(self, axis, command, param=None):
        'Read frequency/firwmare'
        return '{}/{}'.format(self.frequency, self.firmware)

    def axis_command(self, axis, command, param=None):
        'Simulate a command being run on an axis, optionally with a parameter'
        if axis == 0:
            for axis in self.axes:
                self.axis_command(axis, command, param=param)
            # TODO: result?
            return

        command = command.lower()
        try:
            handler = getattr(self, 'handle_{}'.format(command))
        except AttributeError:
            print('TODO', command)
            return b'0'
        else:
            if param is None:
                return handler(axis, command)
            else:
                return handler(axis, command, param=param)

    def received(self, line):
        '''Parse and evaluate a single command

        Forms supported currently:
        {axis_byte}{command}({PARAM})
        {command}({PARAM}):{axis_number}
        {command}:{axis_number}={PARAM}
        '''
        raw_line = line

        if b'=' in line:
            line, param = line.split(b'=')
        else:
            param = None

        if b':' in line:
            line, canbus_target = line.split(b':', 1)
            axis = int(canbus_target)
        elif line[0] in self.addresses:
            axis = self.addresses[line[0]]
            line = line[1:]
        else:
            axis = 1

        if line.endswith(b')') and b'(' in line:
            line, paren_param = line.split(b'(')
            paren_param = paren_param.strip(b')')
            if param is None:
                param = paren_param
            else:
                raise ValueError('hmm')

        line = line.decode('ascii')
        if param is not None:
            param = param.decode('ascii')
        print('<- {} / Axis: {} Command: {} Parameter: {}'
              ''.format(raw_line, axis, line, param))
        return self.axis_command(axis, line, param)


class HgvpuSim(SimState):
    def __init__(self, gap=20 * 1e6):
        super().__init__(positions={1: gap / 2.,
                                    2: gap / 2.,
                                    3: gap / 2.,
                                    4: gap / 2.})

    def handle_gosub(self, axis, command, param=None):
        # self.
        ...


async def handle_sim(sim_state, reader, writer):
    while True:
        try:
            buf = await reader.readuntil(input_delimiter)
        except Exception as ex:
            print('Connection closed: {}', type(ex), ex)
            break

        response = sim_state.received(buf.rstrip(input_delimiter))
        if response is not None:
            if isinstance(response, bytes):
                ...
            elif isinstance(response, str):
                response = response.encode('ascii')
            else:
                response = str(response).encode('ascii')

            if not response.endswith(output_delimiter):
                response += output_delimiter

            print('->', response)
            writer.write(response)
            await writer.drain()


def main(host, port):
    loop = asyncio.get_event_loop()
    # sim_state = SimState(positions={1: 0, 2: 1, 3: 2, 4: 3})
    sim_state = HgvpuSim()
    handler = functools.partial(handle_sim, sim_state)
    coro = asyncio.start_server(handler, host, port, loop=loop)
    server = loop.run_until_complete(coro)

    print('Serving on {}'.format(server.sockets[0].getsockname()))
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        server.close()
        loop.run_until_complete(server.wait_closed())
        loop.close()


if __name__ == '__main__':
    host = '127.0.0.1'
    port = 7001
    main(host, port)
