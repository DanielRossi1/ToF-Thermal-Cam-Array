import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout 

def _cmap(name: str):
    for n in (name, 'plasma', 'viridis'):
        try:
            return pg.colormap.get(n)
        except Exception:
            pass
    return None

class SensorImageView(QWidget):
    """
    GraphicsLayoutWidget with an ImageItem on the left and a
    HistogramLUTItem (colour range editor + gradient bar) on the right.
    """

    def __init__(self, cmap_name: str = 'plasma', parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._glw = pg.GraphicsLayoutWidget()
        layout.addWidget(self._glw)

        self._vb  = self._glw.addViewBox(row=0, col=0)
        self._vb.setAspectLocked(True)
        self._vb.invertY(False)

        self._img = pg.ImageItem()
        self._vb.addItem(self._img)

        self._hist = pg.HistogramLUTItem()
        self._hist.setImageItem(self._img)
        self._glw.addItem(self._hist, row=0, col=1)

        # Give the image 4× more horizontal space than the histogram bar
        self._glw.ci.layout.setColumnStretchFactor(0, 4)
        self._glw.ci.layout.setColumnStretchFactor(1, 1)

        self.set_colormap(cmap_name)

    def set_colormap(self, name: str):
        cm = _cmap(name)
        if cm:
            self._hist.gradient.setColorMap(cm)

    def set_image(self, data2d: np.ndarray):
        """
        data2d: 2-D float array in (cols, rows) order (pyqtgraph convention).
        The histogram range is updated automatically.
        """
        self._img.setImage(data2d, autoLevels=True)
        lo, hi = float(data2d.min()), float(data2d.max())
        if hi > lo:
            self._hist.setLevels(lo, hi)
