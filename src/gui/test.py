import sys
from PyQt6.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsRectItem, QGraphicsItem
from PyQt6.QtGui import QPixmap, QPainter, QPen
from PyQt6.QtCore import Qt, QRectF

class CustomLineItem(QGraphicsItem):
    def __init__(self):
        super().__init__()

    def boundingRect(self):
        return QRectF(0, 0, 200, 200)

    def paint(self, painter, option, widget):
        painter.setPen(QPen(Qt.GlobalColor.blue, 2))
        painter.drawLine(0, 0, 200, 200)
        painter.drawLine(0, 200, 200, 0)

class ZoomableImageWidget(QGraphicsView):
    def __init__(self, image_path):
        super().__init__()

        # Create a QGraphicsScene
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Load the image and add it to the scene
        pixmap = QPixmap(image_path)
        self.image_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(self.image_item)

        # Add a sample sub-widget (a red rectangle)
        rect_item = QGraphicsRectItem(50, 50, 100, 100)
        rect_item.setBrush(Qt.GlobalColor.red)
        self.scene.addItem(rect_item)

        # Add our custom line item
        line_item = CustomLineItem()
        line_item.setPos(100, 100)  # Position the item in the scene
        self.scene.addItem(line_item)

        # Set up the view
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)

    def wheelEvent(self, event):
        # Zoom factor
        zoom_factor = 1.15

        # Zoom in or out
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1 / zoom_factor, 1 / zoom_factor)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = ZoomableImageWidget('../../assets/V5RC-HighStakes-Match-2000x2000.png')
    widget.show()
    sys.exit(app.exec())