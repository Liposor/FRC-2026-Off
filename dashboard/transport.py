"""NT4 transport. Requests are atomic JSON messages, never queued offline."""
import json
import time
import uuid
import ntcore


class Transport:
    def __init__(self, config, demo=False):
        self.config, self.demo = config, demo
        self.inst = ntcore.NetworkTableInstance.create()
        self.session = uuid.uuid4().hex
        self.pending = None
        self.publishers = {}
        self.subscribers = {}
        if not demo:
            self.inst.startClient4('StardustDashboard-' + self.session[:8])
            if config.get('server'):
                self.inst.setServer(config['server'])
            else:
                self.inst.setServerTeam(config['team'])
                self.inst.startDSClient()
        for key in ('enabled', 'voltage', 'mode', 'alliance', 'state', 'ack'):
            path = config['telemetry'][key]
            topic = self.inst.getTopic(path)
            if key == 'enabled':
                self.subscribers[key] = self.inst.getBooleanTopic(path).subscribe(False)
            elif key == 'voltage':
                self.subscribers[key] = self.inst.getDoubleTopic(path).subscribe(0.0)
            else:
                self.subscribers[key] = self.inst.getStringTopic(path).subscribe('')
        self.publishers['request'] = self.inst.getStringTopic(config['request_topic']).publish()

    @property
    def connected(self):
        return self.demo or self.inst.isConnected()

    def read(self):
        if self.demo:
            return dict(enabled=True, voltage=12.6, mode='TELEOP', alliance='AZUL', state='IDLE', ack='')
        return {key: sub.get() for key, sub in self.subscribers.items()}

    def send(self, action, target, level):
        if not self.connected:
            raise RuntimeError('Sem conexão. A solicitação não foi enviada.')
        if self.pending:
            raise RuntimeError('Aguarde a resposta da solicitação anterior.')
        request_id = uuid.uuid4().hex
        payload = dict(id=request_id, session=self.session, action=action,
                       target=target, level=level, issued_at_ms=int(time.time()*1000), ttl_ms=1000)
        if not self.demo:
            self.publishers['request'].set(json.dumps(payload, separators=(',', ':')))
            self.inst.flush()
        self.pending = (request_id, time.monotonic())
        return request_id

    def result(self):
        if not self.pending:
            return None
        rid, started = self.pending
        if self.demo and time.monotonic()-started > .4:
            self.pending = None
            return 'DEMO · ação simulada'
        try:
            ack = json.loads(self.subscribers['ack'].get())
            if ack.get('id') == rid:
                self.pending = None
                return f"Robô · {ack.get('status', 'resposta')} · {ack.get('message', '')}"
        except (ValueError, AttributeError):
            pass
        if not self.connected or time.monotonic()-started > 2:
            self.pending = None
            return 'Sem confirmação do robô. Não reenviado.'
        return None

    def close(self):
        for handle in [*self.publishers.values(), *self.subscribers.values()]:
            handle.close()
        self.inst.stopDSClient()
        self.inst.stopClient()
        ntcore.NetworkTableInstance.destroy(self.inst)
