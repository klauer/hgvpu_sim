'''
Quick simulator of the hgvpu class 5 SmartMotors to aid in debugging and IOC
development.

Usage:
 $ python3 sim.py

Configure asyn on the IOC to connect to localhost, port 7001
'''

import asyncio
import functools
import time

input_delimiter = b' '
output_delimiter = b'\r'


class Axis:
    def __init__(self, index, *, actual_position=0, desired_position=0,
                 torque=0, temperature=22, status=None):

        self.index = index

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
        self.current = 0
        self.velocity = 0
        self.desired_velocity = 1
        self.desired_acceleration = 0
        self.move_mode = 'position'
        self.software_limits = [0, 0]
        self.software_limits_enabled = False
        self.software_limits_fault = True
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self._stopped = False
        self._move_task = None

    def __repr__(self):
        return '<Axis {} Position={}>'.format(self.index,
                                              self.ext_encoder_position)

    def get_status(self, index):
        return self.status.get(index, 0)

    def set_open_loop(self):
        self.closed_loop = False
        self.brake_engaged = True
        self.current = 0
        # TODO: what does the controller respond here?
        return b''

    async def stop(self):
        self._stopped = True

    async def _move(self):
        if not self.move_mode == 'position':
            print('TODO move mode:', self.move_mode)
            return

        self._stopped = False
        self.closed_loop = True
        self.current = 1000

        start_pos = self.actual_position
        desired_pos = self.desired_position
        velocity = self.desired_velocity

        delta_pos = desired_pos - start_pos
        dt = delta_pos / velocity

        t0 = time.monotonic()

        print('\n* Motion from {} to {} at velocity {}; dpos={} dt={}'
              ''.format(start_pos, desired_pos, velocity,
                        delta_pos, dt))

        elapsed = 0
        while elapsed < dt:
            if self.brake_engaged:
                print('Move interrupted: brake engaged')
                return
            elif not self.closed_loop:
                print('Move interrupted: open-loop')
                return
            elif self._stopped:
                print('Move interrupted: stopped')
                return
            self.actual_position = int(start_pos + (elapsed / dt) * delta_pos)
            self.ext_encoder_position = self.actual_position
            print('Step', self.actual_position)
            await asyncio.sleep(0.1)
            elapsed = time.monotonic() - t0

        print('Move complete')
        self.ext_encoder_position = desired_pos
        self.actual_position = desired_pos

    async def move(self, loop):
        self.loop = loop
        if self._move_task is not None:
            print('\n* Canceling other move')
            self._move_task.cancel()

        self._move_task = loop.create_task(self._move())


class SimState:
    addresses = {127 + c: c
                 for c in range(1, 34)}

    def __init__(self, loop, *, positions=None, frequency=8000,
                 firmware='500C'):
        self.loop = loop

        if positions is None:
            positions = {}

        self.axes = {axis: Axis(axis, actual_position=pos,
                                desired_position=pos)
                     for axis, pos in positions.items()
                     }
        self.variables = {}
        self.frequency = frequency
        self._frequency_scale = 65536. / 8000
        self.firmware = firmware

    async def handle_print(self, axis, command, param=None):
        'Read actual position'
        def _print(item):
            if item == 'TEMP':
                return str(axis.temperature)
            elif item.startswith('"') and item.endswith('"'):
                return item.strip('"')
            elif item.startswith('#'):
                num = int(item.lstrip('#'))
                return chr(num)

        return ''.join(_print(item) for item in param.split(','))

    async def handle_rctr(self, axis, command, param=None):
        'Read internal/external encoder position'
        if param is None or int(param) == 0:
            return axis.actual_position
        else:
            return axis.ext_encoder_position

    async def handle_ruia(self, axis, command, param=None):
        'Read current in milliamps'
        return axis.current

    async def handle_rtemp(self, axis, command, param=None):
        'Read temperature'
        return axis.temperature

    async def handle_off(self, axis, command, param=None):
        'Set open loop'
        return axis.set_open_loop()

    async def handle_rpc(self, axis, command, param=None):
        'Read commanded position'
        # TODO: RPC, RPT, and RTA
        return axis.desired_position

    async def handle_rpt(self, axis, command, param=None):
        'Read target position'
        return axis.desired_position

    async def handle_rta(self, axis, command, param=None):
        'Read absolute target position'
        return axis.desired_position

    async def handle_prt(self, axis, command, param=None):
        'Set relative target position'
        axis.desired_position = axis.actual_position + int(param)

    async def handle_pt(self, axis, command, param=None):
        'Set absolute target position'
        axis.desired_position = int(param)

    async def handle_adt(self, axis, command, param=None):
        'Set desired acceleration'
        axis.desired_acceleration = int(param)

    async def handle_vt(self, axis, command, param=None):
        'Set desired velocity'
        axis.desired_velocity = int(param)  # / self._frequency_scale  # TODO

    async def handle_rpa(self, axis, command, param=None):
        'Read absolute actual position'
        return axis.actual_position

    handle_rp = handle_rpa  # TODO?

    async def handle_g(self, axis, command, param=None):
        'Start move / go'
        await axis.move(self.loop)

    async def handle_rt(self, axis, command, param=None):
        'Read torque'
        return axis.torque

    async def handle_mp(self, axis, command, param=None):
        'Move mode: position'
        axis.move_mode = 'position'

    async def handle_mv(self, axis, command, param=None):
        'Move mode: velocity'
        axis.move_mode = 'velocity'

    async def handle_sld(self, axis, command, param=None):
        'Software limit disable'
        axis.software_limits_enabled = False
        return b''

    async def handle_sle(self, axis, command, param=None):
        'Software limit enable'
        axis.software_limits_enabled = True
        return b''

    async def handle_slm(self, axis, command, param=None):
        'Software limit mode'
        axis.software_limits_fault = (int(param) == 1)
        return b''

    async def handle_sln(self, axis, command, param=None):
        'Software low limit'
        axis.software_limits[0] = int(param)
        return b''

    async def handle_slp(self, axis, command, param=None):
        'Software high limit'
        axis.software_limits[1] = int(param)
        return b''

    async def handle_brkrls(self, axis, command, param=None):
        'Brake release'
        axis.brake_engaged = False
        return b''

    async def handle_brkeng(self, axis, command, param=None):
        'Brake engage'
        axis.brake_engaged = True
        return b''

    async def handle_rw(self, axis, command, param=0):
        'Read status word'
        return axis.get_status(int(param))

    async def handle_rsamp(self, axis, command, param=0):
        'Read sampling frequency'
        return self.frequency

    async def handle_rsp(self, axis, command, param=None):
        'Read frequency/firwmare'
        return '{}/{}'.format(self.frequency, self.firmware)

    async def handle_s(self, axis, command, param=0):
        'Stop'
        await axis.stop()

    async def axis_command(self, axis, command, param=None):
        'Simulate a command being run on an axis, optionally with a parameter'
        if axis == 0:
            for axis in self.axes:
                await self.axis_command(axis, command, param=param)
            # TODO: result?
            return

        command = command.lower()
        try:
            handler = getattr(self, 'handle_{}'.format(command))
        except AttributeError:
            print('TODO', command)
            return b'0'
        else:
            axis = self.axes[axis]
            if param is None:
                return await handler(axis, command)
            else:
                return await handler(axis, command, param=param)

    async def received(self, line):
        '''Parse and evaluate a single command

        Forms supported currently:
        {axis_byte}{command}({PARAM})
        {command}({PARAM}):{axis_number}
        {command}:{axis_number}={PARAM}
        '''
        # Stash the line for display later
        raw_line = line

        if b'=' in line:
            # {command}:{axis_number}={PARAM}
            line, param = line.split(b'=')
        else:
            param = None

        if b':' in line:
            # {command}:{axis_number}={PARAM}
            line, canbus_target = line.split(b':', 1)
            axis = int(canbus_target)
        elif line[0] in self.addresses:
            # {axis_byte}{command}({PARAM})
            axis = self.addresses[line[0]]
            line = line[1:]
        else:
            # implicit axis 1
            # {command}({PARAM})
            axis = 1

        if line.endswith(b')') and b'(' in line:
            # {command}({PARAM})
            line, paren_param = line.split(b'(')
            paren_param = paren_param.strip(b')')
            if param is None:
                param = paren_param
            else:
                raise ValueError('hmm')

        line = line.decode('ascii')
        if param is not None:
            param = param.decode('ascii')

        print('<- {} / Axis {} Command {!r} Parameter: {}'
              ''.format(raw_line, axis, line, param), end=' | ')
        return await self.axis_command(axis, line, param)


class HgvpuSim(SimState):
    def __init__(self, loop, *, gap=20 * 1e6):
        super().__init__(loop,
                         positions={1: gap / 2.,
                                    2: gap / 2.,
                                    3: gap / 2.,
                                    4: gap / 2.})
        self.targets = [0, 0, 0, 0]
        self.target_velocity = 1

    async def handle_iii(self, axis, command, param=None):
        'Set target position for m1'
        self.targets[0] = int(param)

    async def handle_jjj(self, axis, command, param=None):
        'Set target position for m2'
        self.targets[1] = int(param)

    async def handle_kkk(self, axis, command, param=None):
        'Set target position for m3'
        self.targets[2] = int(param)

    async def handle_lll(self, axis, command, param=None):
        'Set target position for m4'
        self.targets[3] = int(param)

    async def handle_vvv(self, axis, command, param=None):
        'Set target velocity for gap motion'
        self.target_velocity = int(param)

    async def handle_gosub(self, axis, command, param=None):
        # self.
        ...


async def handle_sim(sim_state, reader, writer):
    while True:
        try:
            buf = await reader.readuntil(input_delimiter)
        except Exception as ex:
            print('Connection closed: {}', type(ex), ex)
            break

        response = await sim_state.received(buf.rstrip(input_delimiter))
        if response is None:
            print()
        else:
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
    # sim_state = SimState(loop, positions={1: 0, 2: 1, 3: 2, 4: 3})
    sim_state = HgvpuSim(loop)
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
