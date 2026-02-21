# ************************************************************************** *#
#                                                                             #
#   Robotic Manipulator Kinematics Analysis Platform                          #
#                                                                             #
#   Written By: Meera Qasem Shersheer  <meera04qasemshersheer@gmail.com>      #
#                                                                             #
#   Institution : University of Jordan — Mechatronics Engineering Dept.       #
#   Course      : Robotic Systems | Instructor: Prof. Zaer Abu Hammour        #
#   Academic Year: 2025/2026 — First Semester                                 #
#                                                                             #
#   Description : A desktop application for forward and inverse kinematics    #
#                 analysis of industrial robotic manipulators (UR5, ABB       #
#                 IRB 1600, KUKA KR16). Supports symbolic and numeric         #
#                 computation modes with interactive 3D visualization.        #
#                                                                             #
#   Convention  : Standard Denavit-Hartenberg (DH) parameters are used        #
#                 throughout. Each frame transformation is defined by         #
#                 four parameters: θ (joint angle), d (link offset),          #
#                 a (link length), and α (link twist). Transformation         #
#                 matrices follow the standard DH formulation.                #
#                                                                             #
#   Copyright (c) 2025 — All Rights Reserved                                  #
#                                                                             #
# ************************************************************************** *#


from imports import *

class NumericDelegate(QStyledItemDelegate):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.validator = QDoubleValidator()
        self.validator.setNotation(QDoubleValidator.Notation.StandardNotation)

    def createEditor(self, parent, option, index):
        editor = QLineEdit(parent)
        editor.setValidator(self.validator)
        editor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        return editor
    
class MatrixDelegate(QStyledItemDelegate):
    def createEditor(self, parent, option, index):
        editor = QLineEdit(parent)
        editor.setValidator(QDoubleValidator())
        editor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        return editor
