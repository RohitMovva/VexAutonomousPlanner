from PyQt6.QtWidgets import QApplication, QDialog, QVBoxLayout, QInputDialog, QMainWindow, QTextEdit, QPushButton, QFrame, QGraphicsScene, QGraphicsView, QGraphicsPixmapItem
from PyQt6.QtGui import QAction, QIcon, QKeySequence, QTransform, QMouseEvent, QColor, QPixmap, QCursor, QBrush
from PyQt6.QtCore import Qt, pyqtSignal, QPoint, QRectF, QPoint

SCALE_FACTOR = 1.25


class PhotoViewer(QGraphicsView):
    coordinatesChanged = pyqtSignal(QPoint)

    def __init__(self, parent):
        super().__init__(parent)
        self._zoom = 0
        self._pinned = False
        self._empty = True
        self._scene = QGraphicsScene(self)
        self._photo = QGraphicsPixmapItem()
        self._photo.setShapeMode(
            QGraphicsPixmapItem.ShapeMode.BoundingRectShape)
        self._scene.addItem(self._photo)
        self.setScene(self._scene)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QBrush(QColor(30, 30, 30)))
        self.setFrameShape(QFrame.Shape.NoFrame)

    # def mousePressEvent(self, event: QMouseEvent):
    #     if event.button() == Qt.MouseButton.LeftButton:
    #         x = int(event.position().x()) + 10 # I have no clue on God's green earth on why this is needed but it is
    #         y = int(event.position().y()) + 10
    #         print(f"Mouse clicked at ({x}, {y})")
    #         self.gui_instance.add_node(x, y)
    #     super().mousePressEvent(event)

    def hasPhoto(self):
        return not self._empty

    def resetView(self, scale=1):
        rect = QRectF(self._photo.pixmap().rect())
        if not rect.isNull():
            self.setSceneRect(rect)
            if (scale := max(1, scale)) == 1:
                self._zoom = 0
            if self.hasPhoto():
                unity = self.transform().mapRect(QRectF(0, 0, 1, 1))
                self.scale(1 / unity.width(), 1 / unity.height())
                viewrect = self.viewport().rect()
                scenerect = self.transform().mapRect(rect)
                factor = min(viewrect.width() / scenerect.width(),
                             viewrect.height() / scenerect.height()) * scale
                self.scale(factor, factor)
                if not self.zoomPinned():
                    self.centerOn(self._photo)
                self.updateCoordinates()

    def setPhoto(self, pixmap=None):
        if pixmap and not pixmap.isNull():
            self._empty = False
            self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
            self._photo.setPixmap(pixmap)
        else:
            self._empty = True
            self.setDragMode(QGraphicsView.DragMode.NoDrag)
            self._photo.setPixmap(QPixmap())
        if not (self.zoomPinned() and self.hasPhoto()):
            self._zoom = 0
        print(SCALE_FACTOR ** self._zoom)
        self.resetView(SCALE_FACTOR ** self._zoom)

    def zoomLevel(self):
        return self._zoom

    def zoomPinned(self):
        return self._pinned

    def setZoomPinned(self, enable):
        self._pinned = bool(enable)

    def zoom(self, step):
        zoom = max(0, self._zoom + (step := int(step)))
        if zoom != self._zoom:
            self._zoom = zoom
            if self._zoom > 0:
                if step > 0:
                    factor = SCALE_FACTOR ** step
                else:
                    factor = 1 / SCALE_FACTOR ** abs(step)
                self.scale(factor, factor)
            else:
                self.resetView()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self.zoom(delta and delta // abs(delta))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.resetView()

    def toggleDragMode(self):
        if self.dragMode() == QGraphicsView.DragMode.ScrollHandDrag:
            self.setDragMode(QGraphicsView.DragMode.NoDrag)
        elif not self._photo.pixmap().isNull():
            self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

    def updateCoordinates(self, pos=None):
        if self._photo.isUnderMouse():
            if pos is None:
                pos = self.mapFromGlobal(QCursor.pos())
            point = self.mapToScene(pos).toPoint()
        else:
            point = QPoint()
        # x = round(((x / (2000)) - 0.5) * 12**2, 2)
        # y = round(((y / (2000)) - 0.5) * 12**2, 2)
        self.coordinatesChanged.emit(point)

    def mouseMoveEvent(self, event):
        self.updateCoordinates(event.position().toPoint())
        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        self.coordinatesChanged.emit(QPoint())
        super().leaveEvent(event)
