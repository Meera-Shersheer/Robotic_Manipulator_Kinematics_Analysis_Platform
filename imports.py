# at the start of the session activate .venv by "source .venv/bin/activate"
# pip install -r requirements.txt   to reinstall all dependancies
# Core numerical computing
import numpy as np
from numpy import sin, cos, tan, arctan2 as atan2, arccos, arcsin, sqrt, pi

# Symbolic mathematics (for deriving and verifying equations)
import sympy as sp
from sympy import symbols, Matrix, simplify, trigsimp

# GUI Framework
import PyQt6
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import QAction, QColor, QPalette, QIcon, QKeySequence, QFont,QPainter


# 3D Visualization
import matplotlib.pyplot as plot
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation


from scipy.spatial.transform import Rotation as R

# For better 3D visualization
import plotly.graph_objects as go
import plotly.express as px

# For robot visualization (highly recommended!)
import roboticstoolbox as rtb
from spatialmath import SE3

# Data handling
import pandas as pd

# System utilities
import sys  #for taking the command line args and exit from the program 
import os
import json # maybe deleted later if not used
from datetime import datetime

from random import randint
from typing import List, Dict, Tuple
from base_manipulator import *
