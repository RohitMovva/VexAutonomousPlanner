import sys
from PyQt6.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsRectItem, QGraphicsItem, QMenu
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor, QBrush, QFont
from PyQt6.QtCore import Qt, QRectF, QPointF, QSize

class Node(QGraphicsItem):
    def __init__(self, x, y, radius=30):
        super().__init__()
        self.x = x
        self.y = y
        self.radius = radius
        self.setPos(x, y)
        self.setAcceptHoverEvents(True)

    def boundingRect(self):
        return QRectF(-self.radius, -self.radius, 2*self.radius, 2*self.radius)

    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Draw the circle
        painter.setPen(QPen(Qt.GlobalColor.black, 2))
        painter.setBrush(QBrush(QColor(173, 216, 230)))  # Light blue color
        painter.drawEllipse(self.boundingRect())

        # Draw the text
        painter.setFont(QFont("Arial", 8))
        painter.setPen(Qt.GlobalColor.black)
        text = f"({self.x:.0f}, {self.y:.0f})"
        painter.drawText(self.boundingRect(), Qt.AlignmentFlag.AlignCenter, text)

    def contextMenuEvent(self, event):
        menu = QMenu()
        action1 = menu.addAction("Action 1")
        action2 = menu.addAction("Action 2")
        menu.exec(event.screenPos())

class ZoomableImageWidget(QGraphicsView):
    def __init__(self, image_path, size=QSize):
        super().__init__()

        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        self.pixmap = QPixmap(image_path)
        self.image_item = QGraphicsPixmapItem(self.pixmap)
        self.scene.addItem(self.image_item)

        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)

        self.fitInView(self.image_item, Qt.AspectRatioMode.KeepAspectRatio)

        self.shift_pressed = False
        self.setMouseTracking(True)

        self.setFixedSize(size)

    def wheelEvent(self, event):
        zoom_factor = 1.15
        old_pos = self.mapToScene(event.position().toPoint())

        current_scale = self.transform().m11()
        new_scale = current_scale * zoom_factor if event.angleDelta().y() > 0 else current_scale / zoom_factor

        if new_scale < self.get_fit_in_view_scale():
            new_scale = self.get_fit_in_view_scale()

        factor = new_scale / current_scale
        self.scale(factor, factor)

        new_pos = self.mapToScene(event.position().toPoint())
        delta = new_pos - old_pos
        self.translate(delta.x(), delta.y())

    def get_fit_in_view_scale(self):
        view_rect = self.viewport().rect()
        scene_rect = self.image_item.boundingRect()
        return min(view_rect.width() / scene_rect.width(), view_rect.height() / scene_rect.height())

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.shift_pressed:
            scene_pos = self.mapToScene(event.position().toPoint())
            node = Node(scene_pos.x(), scene_pos.y())
            node.setPos(scene_pos)
            self.scene.addItem(node)
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        item = self.itemAt(event.position().toPoint())
        if isinstance(item, Node) or (item and isinstance(item.parentItem(), Node)):
            self.setCursor(Qt.CursorShape.ArrowCursor)
        elif self.shift_pressed:
            self.setCursor(Qt.CursorShape.ArrowCursor)
        else:
            self.setCursor(Qt.CursorShape.OpenHandCursor)
        super().mouseMoveEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Shift:
            self.shift_pressed = True
            self.setCursor(Qt.CursorShape.ArrowCursor)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key.Key_Shift:
            self.shift_pressed = False
            self.setCursor(Qt.CursorShape.OpenHandCursor)
        super().keyReleaseEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = ZoomableImageWidget('../../assets/V5RC-HighStakes-Match-2000x2000.png')
    widget.show()
    sys.exit(app.exec())