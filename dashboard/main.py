"""Stardust 9200 • local Python desktop dashboard."""
import argparse
import json
import math
import sys
from pathlib import Path
from PySide6.QtCore import Qt, QTimer, QPointF
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QFrame, QButtonGroup, QCheckBox, QTextEdit)
from transport import Transport

STYLE = '''
QWidget { background: #09090f; color: #eeeaf7; font-family: 'Segoe UI'; font-size: 14px; }
QFrame#card { background: #111119; border: 1px solid #282333; border-radius: 18px; }
QLabel { background: transparent; border: none; }
QLabel#muted { color: #938bA5; font-size: 12px; }
QLabel#title { font-size: 30px; font-weight: 700; }
QLabel#value { font-size: 26px; font-weight: 600; }
QPushButton { background: #191621; border: 1px solid #373042; border-radius: 11px; padding: 14px; font-weight: 600; }
QPushButton:hover { background: #2b203c; border-color: #aa75ff; }
QPushButton:checked { background: #8a4de8; border-color: #bd95ff; color: white; }
QPushButton:pressed { background: #6b34bd; }
QPushButton:disabled { color: #635b71; border-color: #26212e; }
QPushButton#primary { background: #9559f5; color: white; border: none; }
QPushButton#cancel { color: #eda6bc; border-color: #633345; }
QCheckBox { background: transparent; color: #beb3d1; }
QCheckBox::indicator { width: 20px; height: 20px; background: #17121f; border: 1px solid #73568d; border-radius: 5px; }
QCheckBox::indicator:checked { background: #a366ff; border: 2px solid #d8baff; }
QTextEdit { background: #0d0c13; border: 1px solid #272131; border-radius: 10px; color: #b7abc9; font-size: 12px; padding: 8px; }
'''


def label(text, name=None):
    obj = QLabel(text)
    if name: obj.setObjectName(name)
    return obj


def card():
    frame = QFrame()
    frame.setObjectName('card')
    return frame


class Reef(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(410, 380)
        self.group = QButtonGroup(self)
        self.buttons = []
        for text in 'ABCDEFGHIJKL':
            button = QPushButton(text, self)
            button.setCheckable(True)
            button.setFixedSize(58, 52)
            self.group.addButton(button)
            self.buttons.append(button)
        self.buttons[0].setChecked(True)

    def resizeEvent(self, event):
        cx, cy = self.width()/2, self.height()/2
        radius = min(self.width(), self.height())*.40
        for i, button in enumerate(self.buttons):
            angle = math.radians(105-i*30)
            button.move(int(cx+radius*math.cos(angle)-29), int(cy+radius*math.sin(angle)-26))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width()/2, self.height()/2
        radius = min(self.width(), self.height())*.27
        points = [QPointF(cx+radius*math.cos(math.radians(60*i)), cy+radius*math.sin(math.radians(60*i))) for i in range(6)]
        painter.setPen(QPen(QColor('#68469b'), 2))
        painter.setBrush(QColor('#1d142c'))
        painter.drawPolygon(QPolygonF(points))
        painter.setPen(QColor('#c69aff'))
        painter.drawText(self.rect(), Qt.AlignCenter, 'REEF\n9200')

    def selected(self):
        return self.group.checkedButton().text()


class Dashboard(QWidget):
    def __init__(self, config, demo=False):
        super().__init__()
        self.config = config
        self.transport = Transport(config, demo)
        self.demo = demo
        self.setWindowTitle('Stardust 9200 • Dashboard')
        self.resize(1250, 810)
        root = QVBoxLayout(self)
        root.setContentsMargins(30, 24, 30, 24)
        root.setSpacing(18)
        top = QHBoxLayout()
        titlebox = QVBoxLayout()
        titlebox.addWidget(label('STARDUST / 9200', 'muted'))
        titlebox.addWidget(label('Robot Control', 'title'))
        top.addLayout(titlebox)
        top.addStretch()
        self.status = label('●  DESCONECTADO')
        top.addWidget(self.status)
        root.addLayout(top)
        info = QHBoxLayout()
        self.metrics = {}
        for key, caption in [('mode','MODO DO ROBÔ'), ('voltage','BATERIA'), ('alliance','ALIANÇA'), ('state','ESTADO ATUAL')]:
            frame = card(); box = QVBoxLayout(frame)
            box.setContentsMargins(20, 16, 20, 16)
            box.addWidget(label(caption, 'muted'))
            self.metrics[key] = label('—', 'value')
            box.addWidget(self.metrics[key]); info.addWidget(frame)
        root.addLayout(info)
        body = QHBoxLayout(); body.setSpacing(18)
        left = card(); lb = QVBoxLayout(left); lb.setContentsMargins(22,20,22,20)
        lb.addWidget(label('01  /  ESCOLHA O ALVO', 'muted'))
        self.reef = Reef(); lb.addWidget(self.reef,1)
        lb.addWidget(label('Selecione uma posição ao redor do reef.', 'muted'))
        body.addWidget(left, 6)
        middle = card(); mb = QVBoxLayout(middle); mb.setContentsMargins(22,20,22,20)
        mb.addWidget(label('02  /  NÍVEL', 'muted'))
        self.levels = QButtonGroup(self)
        for text in ['L4','L3','L2','L1']:
            btn = QPushButton(text); btn.setCheckable(True); btn.setMinimumHeight(65)
            self.levels.addButton(btn); mb.addWidget(btn)
            if text == 'L2': btn.setChecked(True)
        mb.addStretch(); body.addWidget(middle,2)
        right = card(); rb = QVBoxLayout(right); rb.setContentsMargins(22,20,22,20)
        rb.addWidget(label('03  /  EXECUÇÃO', 'muted'))
        self.selection = label('A  /  L2','value'); rb.addWidget(self.selection)
        hint = label('Solicitações ao robô\nvia NetworkTables 4.','muted'); rb.addWidget(hint)
        rb.addStretch()
        self.arm = QCheckBox('Armar controles'); rb.addWidget(self.arm)
        self.actions = []
        for caption in config['actions']:
            btn = QPushButton(caption)
            btn.setObjectName('primary' if caption == 'Posicionar' else 'cancel' if caption == 'Cancelar' else '')
            btn.clicked.connect(lambda checked=False, c=caption: self.execute(c))
            self.actions.append((caption,btn)); rb.addWidget(btn)
        body.addWidget(right,3); root.addLayout(body,1)
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setFixedHeight(98)
        root.addWidget(self.log)
        footer = QHBoxLayout()
        footer.addWidget(label('NT4  /  roboRIO 2026  /  TEAM '+str(config['team']),'muted'))
        footer.addStretch()
        footer.addWidget(label('DEMONSTRAÇÃO · SEM CONEXÃO REAL' if demo else 'LOCAL · '+(config.get('server') or 'Driver Station / descoberta por equipe'),'muted'))
        root.addLayout(footer)
        self.was_connected = False
        self.record('Modo demonstração: nenhuma mensagem será enviada.' if demo else 'Aguardando roboRIO. Configure os tópicos em config.json.')
        self.timer = QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(100)
        self.refresh()

    def record(self,text):
        from datetime import datetime
        self.log.append(datetime.now().strftime('%H:%M:%S')+'  ·  '+text)

    def execute(self, caption):
        telemetry = self.transport.read()
        if not self.transport.connected or not self.arm.isChecked(): return
        if caption != 'Cancelar' and not telemetry['enabled']: return
        try:
            rid = self.transport.send(self.config['actions'][caption], self.reef.selected(), self.levels.checkedButton().text())
            self.record(f'{caption} · {self.selection.text()} · enviado {rid[:8]} · aguardando confirmação')
        except RuntimeError as exc:
            self.record(str(exc))
        self.refresh()

    def refresh(self):
        connected = self.transport.connected
        if self.was_connected and not connected:
            self.arm.setChecked(False)
            self.record('Conexão perdida. Controles desarmados.')
        self.was_connected = connected
        telemetry = self.transport.read()
        self.status.setText('●  DEMO / LOCAL' if self.demo else '●  NT4 CONECTADO' if connected else '●  DESCONECTADO')
        self.status.setStyleSheet('color: '+('#bf95ff' if connected else '#a99eB6'))
        for key, widget in self.metrics.items():
            value = telemetry[key]
            widget.setText((f'{value:.1f} V' if key=='voltage' and value else str(value or '—')) if connected else '—')
        self.selection.setText(self.reef.selected()+'  /  '+self.levels.checkedButton().text())
        result = self.transport.result()
        if result: self.record(result)
        for caption, button in self.actions:
            button.setEnabled(connected and self.arm.isChecked() and not self.transport.pending and (telemetry['enabled'] or caption=='Cancelar'))
        self.arm.setEnabled(connected)

    def closeEvent(self,event):
        self.timer.stop(); self.transport.close(); event.accept()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo',action='store_true')
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'))
    parser.add_argument('--screenshot',type=Path, help='Captura real da janela em modo demo e encerra.')
    args = parser.parse_args()
    if args.screenshot and not args.demo: parser.error('--screenshot exige --demo')
    config = json.loads(args.config.read_text(encoding='utf-8'))
    app = QApplication(sys.argv); app.setStyle("Fusion"); app.setStyleSheet(STYLE)
    window = Dashboard(config,args.demo); window.show()
    if args.screenshot:
        def capture():
            args.screenshot.parent.mkdir(parents=True,exist_ok=True)
            if not window.grab().save(str(args.screenshot)): raise RuntimeError('Falha ao salvar captura')
            window.close(); app.quit()
        QTimer.singleShot(600,capture)
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
