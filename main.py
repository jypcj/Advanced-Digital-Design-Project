import os
import sys

from PyQt5.QtGui import QFont, QMovie
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QLabel
from PyQt5.QtCore import pyqtSlot, Qt


option = 2

# .py file name
PBF_2D = "\\two\\pbf2d.py"
BAR_2D = "\\two\\bar.py"
BLOCK_2D = "\\two\\block.py"
TWO_BARS_2D = "\\two\\two_bars.py"

PBF_3D = "\\three\\pbf3d.py"
PBF_BLOCK_3D = "\\three\\block.py"
PBF_BOARD_3D = "\\three\\board.py"

# button name
PBF_2D_NAME = "pbf 2d"
BAR_2D_NAME = "bar_2d"
BLOCK_2D_NAME = "block_2d"
TWO_BARS_2D_NAME = "two_bars_2d"

PBF_3D_NAME = "pbf_3d"
BLOCK_3D_NAME = "block_3d"
BOARD_3D_NAME = "board_3d"

class Window(QMainWindow):

    def __init__(self):
        super().__init__()
        self.initUI()


    def initUI(self):
        label = QLabel(self)
        label.setText("Fluid Simulator")
        label.setFixedSize(800, 100)
        label.setFont(QFont("Roman times", 30, QFont.Bold))
        label.move(150, 10)

        # set up the buttons
        # 2D
        pbf_2d_btn = QPushButton("pbf 2D", self)
        pbf_2d_btn.setObjectName(PBF_2D_NAME)
        pbf_2d_btn.clicked.connect(self.click)
        pbf_2d_btn.setFixedSize(150, 100)
        pbf_2d_btn.move(25, 250)

        bar_2d_btn = QPushButton("bar 2D", self)
        bar_2d_btn.setObjectName(BAR_2D_NAME)
        bar_2d_btn.clicked.connect(self.click)
        bar_2d_btn.setFixedSize(150, 100)
        bar_2d_btn.move(225, 250)

        block_2d_btn = QPushButton("block 2D", self)
        block_2d_btn.setObjectName(BLOCK_2D_NAME)
        block_2d_btn.clicked.connect(self.click)
        block_2d_btn.setFixedSize(150, 100)
        block_2d_btn.move(425, 250)

        two_bars_2d_btn = QPushButton("two bars 2D", self)
        two_bars_2d_btn.setObjectName(TWO_BARS_2D_NAME)
        two_bars_2d_btn.clicked.connect(self.click)
        two_bars_2d_btn.setFixedSize(150, 100)
        two_bars_2d_btn.move(625, 250)

        #3D
        pbf_3d_btn = QPushButton("pbf 3D", self)
        pbf_3d_btn.setObjectName(PBF_3D_NAME)
        pbf_3d_btn.clicked.connect(self.click)
        pbf_3d_btn.setFixedSize(150, 100)
        pbf_3d_btn.move(60, 550)

        block_3d_btn = QPushButton("block 3D", self)
        block_3d_btn.setObjectName(BLOCK_3D_NAME)
        block_3d_btn.clicked.connect(self.click)
        block_3d_btn.setFixedSize(150, 100)
        block_3d_btn.move(300, 550)

        board_3d_btn = QPushButton("board 3D", self)
        board_3d_btn.setObjectName(BOARD_3D_NAME)
        board_3d_btn.clicked.connect(self.click)
        board_3d_btn.setFixedSize(150, 100)
        board_3d_btn.move(540, 550)

        # set up window
        self.setWindowTitle("main")
        self.move(200, 100)
        self.setFixedSize(800, 800)
        self.show()


    @pyqtSlot()
    def click(self):
        btn = window.sender()
        btn_name = btn.objectName()
        window.setCursor(Qt.WaitCursor)
        if btn_name == PBF_2D_NAME:
            directory = os.getcwd() + PBF_2D
            os.system("python " + "\"" + directory + "\"")
        elif btn_name == BAR_2D_NAME:
            directory = os.getcwd() + BAR_2D
            os.system("python " + "\"" + directory + "\"")
        elif btn_name == BLOCK_2D_NAME:
            directory = os.getcwd() + BLOCK_2D
            os.system("python " + "\"" + directory + "\"")
        elif btn_name == TWO_BARS_2D_NAME:
            directory = os.getcwd() + TWO_BARS_2D
            os.system("python " + "\"" + directory + "\"")
        elif btn_name == PBF_3D_NAME:
            directory = os.getcwd() + PBF_3D
            os.system("python " + "\"" + directory + "\"")
        elif btn_name == BOARD_3D_NAME:
            directory = os.getcwd() + PBF_BOARD_3D
            os.system("python " + "\"" + directory + "\"")
        elif btn_name == BLOCK_3D_NAME:
            directory = os.getcwd() + PBF_BLOCK_3D
            os.system("python " + "\"" + directory + "\"")

        window.setCursor(Qt.ArrowCursor)
     #   window.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Window()
    window.setWindowTitle("Fluid Simulator")
    window.setObjectName("MainWindow")

    window.setStyleSheet("#MainWindow{border-image:url(./image/bg.png);}")
    app.exec_()


