import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from radar_navigation.base_safety import BaseLease


class BaseSafetyTests(unittest.TestCase):
    def ready(self):
        gate=BaseLease(actuate=True)
        gate.command_at=gate.enable_at=gate.odom_at=gate.hardware_at=10.
        gate.command=(.04,.15)
        return gate

    def test_dry_and_deadlines(self):
        self.assertEqual(BaseLease().output(10),(0.,0.))
        self.assertEqual(self.ready().output(10),(.04,.15))
        for field in ['command_at','enable_at','odom_at','hardware_at']:
            gate=self.ready();setattr(gate,field,9.)
            self.assertEqual(gate.output(10),(0.,0.),field)
        self.assertEqual(self.ready().output(9),(0.,0.))

    def test_caps_and_invalid_input(self):
        gate=self.ready();gate.accept(1.,-1.,10)
        self.assertEqual(gate.output(10),(.05,-.2))
        for v,w,unsupported in [(math.nan,0,False),(0,math.inf,False),(-.01,0,False),(.04,0,True)]:
            gate=self.ready();gate.accept(v,w,10,unsupported)
            self.assertEqual(gate.output(10),(0.,0.))

    def test_invalid_limits(self):
        for kwargs in [dict(max_speed=.2),dict(max_turn=1),dict(command_timeout=0),dict(max_speed=math.nan)]:
            with self.assertRaises(ValueError):BaseLease(**kwargs)

    @unittest.skipUnless(sys.platform.startswith('linux'),'Linux kernel parent-death signal')
    def test_parent_death_kills_child(self):
        with tempfile.TemporaryDirectory() as directory:
            record=Path(directory)/'pid'
            code='''from radar_navigation.base_watchdog import spawn_base
import pathlib,sys,time
p=spawn_base([sys.executable,'-c','import time;time.sleep(60)'])
pathlib.Path(sys.argv[1]).write_text(str(p.pid))
time.sleep(60)
'''
            parent=subprocess.Popen([sys.executable,'-c',code,str(record)])
            child=None
            try:
                deadline=time.monotonic()+3
                while time.monotonic()<deadline and not record.exists():time.sleep(.02)
                self.assertTrue(record.exists())
                child=int(record.read_text())
                self.assertTrue(Path(f'/proc/{child}').exists())
                parent.kill();parent.wait(timeout=2)
                while time.monotonic()<deadline:
                    stat=Path(f'/proc/{child}/stat')
                    try:
                        if stat.read_text().split()[2]=='Z':break
                    except FileNotFoundError:break
                    time.sleep(.02)
                else:self.fail('Base child survived parent SIGKILL')
            finally:
                if parent.poll() is None:parent.kill();parent.wait()
                if child:
                    try:
                        if Path(f'/proc/{child}/stat').read_text().split()[2]!='Z':os.kill(child,signal.SIGKILL)
                    except ProcessLookupError:pass
                    except FileNotFoundError:pass
