# at the start of the session activate .venv by "source .venv/bin/activate"
# pip install -r requirements.txt   to reinstall all dependancies
import numpy as np
from numpy import sin, cos, tan, arctan2 as atan2, arccos, arcsin, sqrt, pi
import sympy as sp
import math
from sympy import symbols, cos, sin, sqrt, atan2, acos, Matrix, pi, simplify, trigsimp, cse
from scipy.spatial.transform import Rotation as R
import PyQt6
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import QAction, QColor, QPalette, QIcon, QKeySequence, QFont,QPainter, QDoubleValidator, QMouseEvent, QPixmap
import trimesh
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt6.QtWebEngineWidgets import QWebEngineView
from scipy.spatial.transform import Rotation as R
import sys
import os
from datetime import datetime
from random import randint
from typing import List, Dict, Tuple
from base_manipulator import *
from input_constrains import *
