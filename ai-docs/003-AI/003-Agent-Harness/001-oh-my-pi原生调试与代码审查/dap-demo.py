"""Minimal DAP reproduction: Python 3 standard library + clang + lldb-dap."""
import asyncio
import json
import os
from pathlib import Path

async def main():
    root = Path(__file__).resolve().parent
    source = root / 'demo.c'
    source.write_text('int main(void) {\n  int values[3] = {10, 20, 30};\n  int i = 3;\n  return values[i];\n}\n')
    build = await asyncio.create_subprocess_exec('clang', '-g', '-O0', str(source), '-o', str(root / 'demo'))
    assert await build.wait() == 0
    proc = await asyncio.create_subprocess_exec('lldb-dap', stdin=asyncio.subprocess.PIPE,
                                                stdout=asyncio.subprocess.PIPE, stderr=None, env={**os.environ, 'DEBUGINFOD_URLS': ''})
    pending, events, seq = {}, asyncio.Queue(), 0
    async def reader():
        try:
            while True:
                header = await proc.stdout.readuntil(b'\r\n\r\n')
                size = int(next(x.split(b':', 1)[1] for x in header.split(b'\r\n') if x.lower().startswith(b'content-length:')))
                msg = json.loads(await proc.stdout.readexactly(size))
                if msg['type'] == 'response':
                    future = pending.pop(msg['request_seq'], None)
                    if future is not None and not future.done():
                        if msg['success']: future.set_result(msg.get('body', {}))
                        else: future.set_exception(RuntimeError(msg.get('message', str(msg))))
                elif msg['type'] == 'event': await events.put(msg)
        except (asyncio.IncompleteReadError, asyncio.CancelledError):
            for f in pending.values():
                if not f.done(): f.set_exception(RuntimeError('Adapter closed'))
    async def request(command, **args):
        nonlocal seq
        seq += 1
        request_id = seq
        future = asyncio.get_running_loop().create_future()
        pending[request_id] = future
        data = json.dumps(dict(seq=request_id, type='request', command=command, arguments=args)).encode()
        proc.stdin.write(f'Content-Length: {len(data)}\r\n\r\n'.encode() + data)
        await proc.stdin.drain()
        print('SEND', command)
        try: return await asyncio.wait_for(future, 30)
        finally: pending.pop(request_id, None)
    async def event(name):
        while True:
            msg = await asyncio.wait_for(events.get(), 30)
            if msg['event'] == name:
                print('EVENT', name, {k:v for k,v in msg.get('body', {}).items() if not k.startswith('$')})
                return msg.get('body', {})
    pump = asyncio.create_task(reader())
    launch = None
    try:
        caps = await request('initialize', adapterID='lldb', clientID='minimal-review',
                             linesStartAt1=True, columnsStartAt1=True, pathFormat='path')
        # launch may wait for configurationDone: it must stay in flight.
        launch = asyncio.create_task(request('launch', program=str(root / 'demo'), cwd=str(root), stopOnEntry=False, initCommands=['settings set symbols.enable-external-lookup false']))
        await event('initialized')
        bp = await request('setBreakpoints', source={'path': str(source)}, breakpoints=[{'line': 4}])
        assert bp['breakpoints'][0]['verified']
        if caps.get('supportsConfigurationDoneRequest'): await request('configurationDone')
        await launch
        stop = await event('stopped')
        frames = await request('stackTrace', threadId=stop['threadId'], levels=5)
        frame = frames['stackFrames'][0]
        scopes = await request('scopes', frameId=frame['id'])
        locals_ = await request('variables', variablesReference=scopes['scopes'][0]['variablesReference'])
        result = await request('evaluate', expression='i', frameId=frame['id'], context='watch')
        print('FRAME', frame['name'], 'line', frame['line'])
        print('LOCALS', json.dumps(locals_['variables']))
        print('EVALUATE i =', result['result'])
        assert frame['line'] == 4 and result['result'] == '3'
        print('EVIDENCE: index 3 is outside values[3]; stopped before the invalid access.')
        await request('disconnect', terminateDebuggee=True)
    finally:
        if launch is not None:
            if not launch.done(): launch.cancel()
            await asyncio.gather(launch, return_exceptions=True)
        if proc.returncode is None:
            try: await asyncio.wait_for(proc.wait(), 2)
            except TimeoutError: proc.kill(); await proc.wait()
        pump.cancel()
        await asyncio.gather(pump, return_exceptions=True)

asyncio.run(main())
