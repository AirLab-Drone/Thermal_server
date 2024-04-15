#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
import sys



class TextDisplay(QWidget):
    def __init__(self, text):
        super().__init__()
        self.setWindowTitle('Text Display')
        self.setGeometry(100, 100, 400, 200)
        
        layout = QVBoxLayout()
        self.label = QLabel()
        self.label.setText(text)
        layout.addWidget(self.label)
        
        self.setLayout(layout)

if __name__ == '__main__':
    pass
