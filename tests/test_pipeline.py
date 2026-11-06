import io
import json
import math
import struct
import tempfile
import threading
import unittest
import urllib.request
from pathlib import Path
from radar_navigation.protocol import Parser, Frame, MAGIC
from radar_navigation.core import Filter, Grid, Pose, ros_points
from radar_navigation.fusion import Fusion, calibrate
from radar_navigation.io import write_frame, replay
from radar_navigation.planning import astar


def packet(points=((1.,2.,.3,.1),), number=7):
    payload = b''.join(struct.pack('<4f', *p) for p in points)
    return MAGIC+struct.pack('<8I',1,48+len(payload),0,number,123,len(points),1,0)+struct.pack('<2I',1,len(payload))+payload


class PipelineTests(unittest.TestCase):
    def test_every_split(self):
        data = packet()
        for split in range(len(data)+1):
            parser = Parser()
            frames = parser.feed(data[:split],10)+parser.feed(data[split:],10)
            self.assertEqual(len(frames),1)
            self.assertEqual(frames[0].number,7)
            self.assertAlmostEqual(frames[0].points[0][3],.1)
            self.assertEqual(frames[0].metadata['cycles'],123)

    def test_bad_tlv_recovers(self):
        data = bytearray(packet())
        struct.pack_into('<I', data,44,10000)
        parser = Parser()
        result = parser.feed(b'garbage'+data+packet(number=8))
        self.assertEqual([f.number for f in result],[8])
        self.assertEqual(parser.rejected,1)

    def test_bounded_garbage(self):
        parser = Parser()
        parser.feed(b'x'*1000000)
        self.assertLessEqual(len(parser.buffer),7)

    def test_side_information(self):
        raw = bytearray(packet())
        raw.extend(struct.pack('<2I2H',7,4,200,150))
        struct.pack_into('<I',raw,12,len(raw))
        struct.pack_into('<I',raw,32,2)
        self.assertEqual(Parser().feed(raw)[0].metadata['side_info_db'],[(20.,15.)])

    def test_record_replay(self):
        f = Frame('front',123,1,[[1,2,3,4]],{'cycles':99})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'frames.jsonl'
            with path.open('w') as stream:
                write_frame(stream,f)
            self.assertEqual(list(replay(path,0))[0],f)

    def test_transform_and_doppler(self):
        f=Frame('a',0,1,[[1,2,3,4]])
        self.assertEqual(ros_points(f),[(2,-1,3,4)])
        point=Pose(1,2,math.pi/2).apply((2,-1,3,4))
        self.assertAlmostEqual(point[0],2)
        self.assertAlmostEqual(point[1],4)
        self.assertEqual(point[3],4)

    def test_filter_and_expiry(self):
        filt=Filter(confirmations=2)
        f=Frame('a',0,1,[[0,2,.3,0],[0,float('nan'),0,0]])
        self.assertEqual(filt.update(f),[])
        f.stamp_ns=100000000
        self.assertEqual(len(filt.update(f)),1)
        self.assertEqual(filt.update(Frame('a',1000000000,3,[])),[])

    def test_dynamic_retained_immediately(self):
        self.assertEqual(len(Filter(confirmations=3).update(Frame('a',0,0,[[0,2,.3,.4]]))),1)

    def test_unknown_and_clear(self):
        grid=Grid(width=20,height=20,resolution=1,origin=(0,0))
        grid.update([(5.5,5.5,0,0)],(1.5,5.5))
        self.assertEqual(grid.values()[grid.index((3,5))],-1)
        self.assertGreater(grid.values()[grid.index((5,5))],65)
        grid.clear=True
        grid.update([(5.5,5.5,0,0)],(1.5,5.5))
        self.assertLess(grid.values()[grid.index((3,5))],50)
        self.assertIsNone(grid.index(grid.cell(-.01,0)))

    def test_planner_unknown_and_obstacle(self):
        grid=Grid(width=10,height=10,resolution=1,origin=(0,0))
        self.assertEqual(astar(grid,(1,1),(8,1)),[])
        grid.update([(4.5,1.5,0,0)])
        path=astar(grid,(1,1),(8,1),radius=0,allow_unknown=True)
        self.assertTrue(path)
        self.assertNotIn((4,1),path)

    def test_calibration(self):
        pose,error=calibrate([(0,0),(1,0),(0,1)],[(2,3),(2,4),(1,3)])
        self.assertLess(error,1e-10)
        self.assertAlmostEqual(pose.yaw,math.pi/2)

    def test_fusion_expiry(self):
        fusion=Fusion()
        fusion.update('a',0,[(1,0,0,0)])
        self.assertEqual(len(fusion.update('b',.1,[(2,0,0,0)])),2)
        self.assertEqual(len(fusion.update('b',1,[(2,0,0,0)])),1)

    def test_simulated_navigation(self):
        from radar_navigation.simulate import trial
        result=trial(seed=0,beams=61)
        self.assertTrue(result['success'])
        self.assertEqual(result['collisions'],0)
        self.assertLessEqual(result['path_efficiency'],1.)

    def test_corrupt_count_and_empty_frame(self):
        raw=bytearray(packet())
        struct.pack_into('<I',raw,28,99)
        parser=Parser()
        frames=parser.feed(raw+packet(points=(),number=8))
        self.assertEqual(len(frames),1)
        self.assertEqual(frames[0].points,[])
        self.assertEqual(parser.rejected,1)

    def test_viewer_http_sse(self):
        import radar_viewer as v
        server=v.ThreadingHTTPServer(('127.0.0.1',0),v.Handler)
        server.latest=v.LatestFrame()
        server.latest.publish(json.dumps({'frame':42,'points':[[1,2,3,4]]}))
        thread=threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        try:
            base='http://127.0.0.1:'+str(server.server_port)
            with urllib.request.urlopen(base,timeout=2) as response:
                self.assertIn(b'AWR1843 live point cloud',response.read())
            with urllib.request.urlopen(base+'/stream',timeout=2) as response:
                self.assertEqual(json.loads(response.readline()[6:])['frame'],42)
        finally:
            server.shutdown()
            server.server_close()


if __name__=='__main__':
    unittest.main()
