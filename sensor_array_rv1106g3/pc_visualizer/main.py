#!/usr/bin/env python3

import sys, argparse, os

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, 
)
from PyQt5.QtGui  import QColor, QPalette

import pyqtgraph as pg
from config.defaults import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_PROTO,
)

from main_window import MainWindow

def main():
    parser = argparse.ArgumentParser(description='Sensor Hub Visualizer v2')
    parser.add_argument('--host',  default=DEFAULT_HOST,
                        help='Device IP address (default: %(default)s)')
    parser.add_argument('--port',  type=int, default=DEFAULT_PORT,
                        help='TCP/UDP port (default: %(default)s)')
    parser.add_argument('--proto', default=DEFAULT_PROTO,
                        choices=['TCP', 'UDP'],
                        help='Transport protocol (default: %(default)s)')
    args = parser.parse_args()

    # pyqtgraph global config
    pg.setConfigOption('background', '#0d0d1a')
    pg.setConfigOption('foreground', '#d0d0d0')
    pg.setConfigOption('antialias',  True)

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Deep blue-black dark palette
    pal = QPalette()
    c = {
        QPalette.Window:          QColor( 13,  13,  26),
        QPalette.WindowText:      QColor(220, 220, 220),
        QPalette.Base:            QColor(  8,   8,  20),
        QPalette.AlternateBase:   QColor( 22,  22,  40),
        QPalette.ToolTipBase:     QColor(200, 200, 200),
        QPalette.ToolTipText:     QColor( 20,  20,  30),
        QPalette.Text:            QColor(215, 215, 215),
        QPalette.Button:          QColor( 35,  35,  65),
        QPalette.ButtonText:      QColor(215, 215, 215),
        QPalette.BrightText:      QColor(255, 255, 255),
        QPalette.Highlight:       QColor( 55,  90, 180),
        QPalette.HighlightedText: QColor(255, 255, 255),
        QPalette.Link:            QColor( 79, 195, 247),
        QPalette.LinkVisited:     QColor(149, 117, 205),
    }
    for role, colour in c.items():
        pal.setColor(role, colour)
    app.setPalette(pal)

    win = MainWindow(host=args.host, port=args.port, proto=args.proto)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()