import json
import os
import sys
import time
import unittest
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import ntcore
from transport import Transport
from main import Dashboard
from PySide6.QtWidgets import QApplication
CONFIG = json.loads((Path(__file__).resolve().parents[1]/'config.json').read_text())


def wait(predicate):
    until = time.monotonic()+3
    while time.monotonic()<until:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('NT4 não respondeu')


class Tests(unittest.TestCase):
    def test_nt4_roundtrip(self):
        server = ntcore.NetworkTableInstance.create()
        server.startServer('', '127.0.0.1', 11735, 15810)
        cfg = dict(CONFIG, server='127.0.0.1')
        client = Transport(cfg)
        client.inst.setServer('127.0.0.1',15810)
        req = server.getStringTopic(cfg['request_topic']).subscribe('')
        ack = server.getStringTopic(cfg['telemetry']['ack']).publish()
        try:
            wait(lambda:client.connected)
            rid = client.send('position','A','L2')
            wait(lambda:bool(req.get()))
            payload = json.loads(req.get())
            self.assertEqual(payload['id'],rid)
            self.assertEqual(payload['target'],'A')
            with self.assertRaises(RuntimeError): client.send('intake','A','L2')
            ack.set(json.dumps(dict(id='wrong',status='accepted')));server.flush()
            time.sleep(.05);self.assertIsNone(client.result())
            ack.set(json.dumps(dict(id=rid,status='accepted',message='ok')));server.flush()
            results=[]
            def received():
                result=client.result()
                if result: results.append(result)
                return bool(results)
            wait(received)
            self.assertIn('accepted',results[0])
            rid2=client.send('position','A','L2');self.assertNotEqual(rid,rid2)
            client.pending=(rid2,time.monotonic()-3)
            self.assertIn('Sem confirmação',client.result())
            server.stopServer();wait(lambda:not client.connected)
            with self.assertRaises(RuntimeError):client.send('intake','A','L2')
        finally:
            client.close();req.close();ack.close();server.stopServer()
            ntcore.NetworkTableInstance.destroy(server)

    def test_ui_selection_and_arming(self):
        app=QApplication.instance() or QApplication([])
        window=Dashboard(CONFIG,True)
        try:
            self.assertTrue(all(not b.isEnabled() for _,b in window.actions))
            window.arm.setChecked(True);window.refresh()
            self.assertTrue(all(b.isEnabled() for _,b in window.actions))
            window.reef.buttons[5].click();window.refresh()
            self.assertIn('F',window.selection.text())
            window.execute('Posicionar')
            self.assertIsNotNone(window.transport.pending)
            self.assertTrue(all(not b.isEnabled() for _,b in window.actions))
        finally:window.close()


if __name__=='__main__':unittest.main()
