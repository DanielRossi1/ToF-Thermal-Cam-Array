import collections
import time
from PyQt5.QtWidgets import QFormLayout, QGroupBox, QLabel, QVBoxLayout
import pyqtgraph as pg
from network.protocol import  SyncedFrame

class StatsWidget(QGroupBox):
    _HIST = 90   # sparkline width in samples

    def __init__(self, parent=None):
        super().__init__('Statistics', parent)
        layout = QVBoxLayout(self)

        form = QFormLayout()
        self._fps_lbl  = QLabel('0.0')
        self._seq_lbl  = QLabel('—')
        self._drop_lbl = QLabel('0')
        self._tof_lbl  = QLabel('—')
        self._mlx_lbl  = QLabel('—')
        self._cam_lbl  = QLabel('—')

        for lbl, w in [('FPS:', self._fps_lbl),     ('Seq:', self._seq_lbl),
                        ('Dropped:', self._drop_lbl), ('ToF:', self._tof_lbl),
                        ('MLX:', self._mlx_lbl),     ('Camera:', self._cam_lbl)]:
            form.addRow(lbl, w)
        layout.addLayout(form)

        # FPS sparkline
        self._spark = pg.PlotWidget()
        self._spark.setMaximumHeight(90)
        self._spark.setBackground('#0a0a14')
        self._spark.hideAxis('bottom')
        self._spark.showGrid(y=True, alpha=0.25)
        self._spark.setYRange(0, 35)
        self._spark.setLabel('left', 'FPS', color='#aaa', size='8pt')
        self._curve = self._spark.plot(
            pen=pg.mkPen(color='#4fc3f7', width=2))
        self._fill  = pg.FillBetweenItem(
            self._curve,
            self._spark.plot([0] * self._HIST, pen=None),
            brush=pg.mkBrush(79, 195, 247, 40))
        self._spark.addItem(self._fill)
        layout.addWidget(self._spark)

        self._times     = collections.deque(maxlen=300)
        self._fps_hist  = collections.deque([0.0] * self._HIST, maxlen=self._HIST)
        self._prev_seq  = None
        self._drops     = 0

    def update(self, sf: SyncedFrame):
        now = time.monotonic()
        self._times.append(now)

        window = [t for t in self._times if now - t < 2.0]
        fps    = len(window) / 2.0
        self._fps_hist.append(fps)
        self._fps_lbl.setText(f'{fps:.1f}')
        self._curve.setData(list(self._fps_hist))

        if self._prev_seq is not None:
            gap = int((sf.seq - self._prev_seq - 1) & 0xFFFF_FFFF)
            if gap:
                self._drops += gap
                self._drop_lbl.setText(str(self._drops))
        self._prev_seq = sf.seq
        self._seq_lbl.setText(str(sf.seq))

        self._tof_lbl.setText('✓' if sf.tof else '✗')
        self._mlx_lbl.setText('✓' if sf.mlx else '✗')
        self._cam_lbl.setText(
            f'✓  {len(sf.cam_jpeg)//1024} KiB' if sf.cam_jpeg else '✗')

