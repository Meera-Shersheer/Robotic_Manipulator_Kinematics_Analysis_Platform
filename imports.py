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

# at the start of the session activate .venv by "source .venv/bin/activate"
# pip install -r requirements.txt   to reinstall all dependancies
import numpy as np
from numpy import sin, cos, tan, arctan2 as atan2, arccos, arcsin, sqrt, pi
import sympy as sp
import math
from sympy import Symbol, Matrix, simplify, latex, sympify
import PyQt6
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import QAction, QColor, QPalette, QIcon, QKeySequence, QFont,QPainter, QDoubleValidator, QMouseEvent, QPixmap
import trimesh
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt6.QtWebEngineWidgets import QWebEngineView
import sys
import os
from datetime import datetime
from random import randint
from typing import List, Dict, Tuple
from base_manipulator import *
from input_constrains import *
import ctypes