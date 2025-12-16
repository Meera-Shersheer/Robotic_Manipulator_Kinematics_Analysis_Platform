# from imports import *

# class MainWindow(QMainWindow): #defining our class (inheriting from QMainWindow)
#     def __init__(self):    #Constructor
#         super().__init__()  # calling the parent constructor

#         self.setWindowTitle("Robotics IK/FK Calculator")   # giving a title to the window 
#         self.resize(1400, 1000) # resizing the window
       
#         central = QWidget()  # Create a central widget (required in QMainWindow)
#         self.setCentralWidget(central)

#         # Main layout for the central widget
#         layout = QVBoxLayout()
#         central.setLayout(layout)

#         # Title
#         title = QLabel("Robotics IK/FK Platform")
#         title.setStyleSheet("font-size:28px; font-weight:bold; font-family:Times New Roman;")
#         title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
#         layout.addWidget(title)
        
#         ## input 
#         widget = QLineEdit()
# #        widget.setMaxLength(10)
#         widget.setPlaceholderText("Enter your text")
#         layout.addWidget(widget)
        
#         # layout.addWidget(QLabel("Robot Selection Panel"))
#         # layout.addWidget(QLabel("DH Parameter Panel"))
#         # layout.addWidget(QLabel("FK/IK Outputs Panel"))
#         # layout.addWidget(QLabel("3D Visualization Panel"))         

#     #     manipulators_list = QComboBox()
#     #     manipulators_list.addItems(["One", "Two", "Three"])
#     # # Sends the current index (position) of the selected item.
#     #     manipulators_list.currentIndexChanged.connect( self.index_changed )
#     # # There is an alternate signal to send the text.
#     #     manipulators_list.currentTextChanged.connect( self.text_changed )
#     #     layout.addWidget(manipulators_list)

#     #     widget = QListWidget()
#     #     widget.addItems(["One", "Two", "Three"])

#     #     widget.currentItemChanged.connect(self.index_changed)
#     #     widget.currentTextChanged.connect(self.text_changed)
#     #     layout.addWidget(widget)
    
    
# # trying signals 
#         # self.button_is_checked = True
#         # button = QPushButton("Press Me!")
#         # button.setCheckable(True)
#         # button.clicked.connect(self.the_button_was_toggled)
#         # button.setChecked(self.button_is_checked)
#         # layout.addWidget(button)


#     # def the_button_was_clicked(self):
#     #     print("Clicked!")
#     def the_button_was_toggled(self, checked):
#         self.button_is_checked = checked

#         print(self.button_is_checked)

# #Right click menu
#     def contextMenuEvent(self, e):
#         context = QMenu(self)
#         context.addAction(QAction("test 1", self))
#         context.addAction(QAction("test 2", self))
#         context.addAction(QAction("test 3", self))
#         context.exec(e.globalPos())

# ## other way to work with menus
# # class MainWindow(QMainWindow):
# #     def __init__(self):
# #         super().__init__()
# #         self.show()

# #         self.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
# #         self.customContextMenuRequested.connect(self.on_context_menu)

# #     def on_context_menu(self, pos):
# #         context = QMenu(self)
# #         context.addAction(QAction("test 1", self))
# #         context.addAction(QAction("test 2", self))
# #         context.addAction(QAction("test 3", self))
# #         context.exec(self.mapToGlobal(pos))


#     def index_changed(self, i): # i is an int
#         print(i)

#     def text_changed(self, s): # s is a str
#         print(s)

# class Color(QWidget):
#     def __init__(self, color):
#         super().__init__()
#         self.setAutoFillBackground(True)

#         palette = self.palette()
#         palette.setColor(QPalette.ColorRole.Window, QColor(color))
#         self.setPalette(palette)
        
# class MainWindow(QMainWindow):

#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("My App")

#        # layout = QVBoxLayout()
#         layout = QHBoxLayout()

#         layout.addWidget(Color("red"))
#         layout.addWidget(Color("green"))
#         layout.addWidget(Color("orange"))
#         layout.addWidget(Color("blue"))

#         widget = QWidget()
#         widget.setLayout(layout)
#         self.setCentralWidget(widget)

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("My App")

#         layout1 = QHBoxLayout()
#         layout2 = QVBoxLayout()
#         layout3 = QVBoxLayout()

#         layout2.addWidget(Color("red"))
#         layout2.addWidget(Color("yellow"))
#         layout2.addWidget(Color("purple"))

#         layout1.setContentsMargins(0,0,0,0)
#         layout1.setSpacing(20)
        
#         layout1.addLayout(layout2)

#         layout1.addWidget(Color("green"))

#         layout3.addWidget(Color("orange"))
#         layout3.addWidget(Color("blue"))

#         layout1.addLayout(layout3)

#         widget = QWidget()
#         widget.setLayout(layout1)
#         self.setCentralWidget(widget)


# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("My App")

#         layout = QGridLayout()

#         layout.addWidget(Color("red"), 0, 3)
#         layout.addWidget(Color("green"), 1, 1)
#         layout.addWidget(Color("orange"), 2, 2)
#         layout.addWidget(Color("blue"), 3, 0)

#         widget = QWidget()
#         widget.setLayout(layout)
#         self.setCentralWidget(widget)



# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("My App")

#         pagelayout = QVBoxLayout()
#         button_layout = QHBoxLayout()
#         self.stacklayout = QStackedLayout()

#         pagelayout.addLayout(button_layout)
#         pagelayout.addLayout(self.stacklayout)

#         btn = QPushButton("red")
#         btn.pressed.connect(self.activate_tab_1)
#         button_layout.addWidget(btn)
#         self.stacklayout.addWidget(Color("red"))

#         btn = QPushButton("green")
#         btn.pressed.connect(self.activate_tab_2)
#         button_layout.addWidget(btn)
#         self.stacklayout.addWidget(Color("green"))

#         btn = QPushButton("yellow")
#         btn.pressed.connect(self.activate_tab_3)
#         button_layout.addWidget(btn)
#         self.stacklayout.addWidget(Color("yellow"))

#         widget = QWidget()
#         widget.setLayout(pagelayout)
#         self.setCentralWidget(widget)

#     def activate_tab_1(self):
#         self.stacklayout.setCurrentIndex(0)

#     def activate_tab_2(self):
#         self.stacklayout.setCurrentIndex(1)

#     def activate_tab_3(self):
#         self.stacklayout.setCurrentIndex(2)


# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         self.setWindowTitle("My App")

#         tabs = QTabWidget()
#         tabs.setTabPosition(QTabWidget.TabPosition.West)
#         tabs.setMovable(True)

#         for color in ["red", "green", "blue", "yellow"]:
#             tabs.addTab(Color(color), color)

#         self.setCentralWidget(tabs)

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("My App")

#         label = QLabel("Hello!")
#         label.setAlignment(Qt.AlignmentFlag.AlignCenter)

#         self.setCentralWidget(label)

#         toolbar = QToolBar("My main toolbar")
#         toolbar.setIconSize(QSize(16, 16))
#         self.addToolBar(toolbar)

#         button_action = QAction(QIcon("arrow-000-small.png"), "&Your button", self)
#         self.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
#         button_action.setStatusTip("This is your button")
#         button_action.triggered.connect(self.toolbar_button_clicked)
#         button_action.setCheckable(True)
#         button_action.setShortcut(QKeySequence("Ctrl+p"))
#         toolbar.addAction(button_action)

#         toolbar.addSeparator()

#         button_action2 = QAction(QIcon("arrow-000-small.png"), "Your &button2", self)
#         button_action2.setStatusTip("This is your button2")
#         button_action2.triggered.connect(self.toolbar_button_clicked)
#         button_action2.setCheckable(True)
#         toolbar.addAction(button_action2)

#         toolbar.addWidget(QLabel("Hello"))
#         toolbar.addWidget(QCheckBox())
#         self.setStatusBar(QStatusBar(self))

#         menu = self.menuBar()

#         file_menu = menu.addMenu("&File")
#         file_menu.addAction(button_action)
#         file_menu.addSeparator()
    
#         file_submenu = file_menu.addMenu("Submenu")
#         file_submenu.addAction(button_action2)
        
#     def toolbar_button_clicked(self, s):
#         print("click", s) 


# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         self.setWindowTitle("My App")
#         self.resize(1400, 1000) # resizing the window

#         button = QPushButton("Press me for a dialog!")
#         button.clicked.connect(self.button_clicked)
#         self.setCentralWidget(button)

#     def button_clicked(self, s):
#         print("click", s)

#         dlg = CustomDialog()
#         if dlg.exec():
#             print("Success!")
#         else:
#             print("Cancel!")


# class CustomDialog(QDialog):
#     def __init__(self):
#         super().__init__()

#         self.setWindowTitle("HELLO!")

#         QBtn = (
#             QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
#         )

#         self.buttonBox = QDialogButtonBox(QBtn)
#         self.buttonBox.accepted.connect(self.accept)
#         self.buttonBox.rejected.connect(self.reject)

#         layout = QVBoxLayout()
#         message = QLabel("Something happened, is that OK?")
#         layout.addWidget(message)
#         layout.addWidget(self.buttonBox)
#         self.setLayout(layout)

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         self.setWindowTitle("My App")

#         button = QPushButton("Press me for a dialog!")
#         button.clicked.connect(self.button_clicked)
#         self.setCentralWidget(button)

    # def button_clicked(self, s):
    #     dlg = QMessageBox(self)
    #     dlg.setWindowTitle("I have a question!")
    #     dlg.setText("This is a simple dialog")
    #     button = dlg.exec()

    #     if button == QMessageBox.StandardButton.Ok:
    #         print("OK!")
  
    # def button_clicked(self, s):
    #     button = QMessageBox.question(
    #         self,
    #         "Question dialog",
    #         "The longer message"
    #     )

    #     if button == QMessageBox.StandardButton.Yes:
    #         print("Yes!")
    #     else:
    #         print("No!")  
  
    # def button_clicked(self, s):
    #     button = QMessageBox.critical(
    #         self,
    #         "Oh dear!",
    #         "Something went very wrong.",
    #         buttons=QMessageBox.StandardButton.Discard
    #         | QMessageBox.StandardButton.NoToAll
    #         | QMessageBox.StandardButton.Ignore,
    #         defaultButton=QMessageBox.StandardButton.Discard,
    #     )

    #     if button == QMessageBox.StandardButton.Discard:
    #         print("Discard!")
    #     elif button == QMessageBox.StandardButton.NoToAll:
    #         print("No to all!")
    #     else:
    #         print("Ignore!")

    # def button_clicked(self, s):
    #     dlg = QMessageBox(self)
    #     dlg.setWindowTitle("I have a question!")
    #     dlg.setText("This is a question dialog")
    #     dlg.setStandardButtons(
    #         QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
    #     )
    #     dlg.setIcon(QMessageBox.Icon.Question)
    #     button = dlg.exec()

    #     if button == QMessageBox.StandardButton.Yes:
    #         print("Yes!")
    #     else:
    #         print("No!")
## QMessageBox.about(parent, title, message)
## QMessageBox.critical(parent, title, message)
## QMessageBox.information(parent, title, message)
## QMessageBox.question(parent, title, message)
## QMessageBox.warning(parent, title, message)

# class AnotherWindow(QWidget):
#     """
#     This "window" is a QWidget. If it has no parent, it
#     will appear as a free-floating window as we want.
#     """
#     def __init__(self):
#         super().__init__()
#         layout = QVBoxLayout()
#         self.label = QLabel("Another Window % d" % randint(0,100))
#         layout.addWidget(self.label)
#         self.setLayout(layout)


# class MainWindow(QMainWindow):

#     def __init__(self):
#         super().__init__()
#         self.w = None  # No external window yet.
#         self.button = QPushButton("Push for Window")
#         self.button.clicked.connect(self.show_new_window)
#         self.setCentralWidget(self.button)

#     # def show_new_window(self, checked):
#     #     if self.w is None:
#     #         self.w = AnotherWindow()
#     #     self.w.show()
#     def show_new_window(self, checked):
#         if self.w is None:
#             self.w = AnotherWindow()
#             self.w.show()

#         else:
#             self.w.close()  # Close window.
#             self.w = None  # Discard reference, close window.



# class MainWindow(QMainWindow):

#     def __init__(self):
#         super().__init__()
#         self.w = AnotherWindow()
#         self.button = QPushButton("Push for Window")
#         # self.button.clicked.connect(self.show_new_window)
#         self.button.clicked.connect(self.toggle_window)
#         self.setCentralWidget(self.button)

#     def show_new_window(self, checked):
#         self.w.show()
    
#     def toggle_window(self, checked):
#         if self.w.isVisible():
#             self.w.hide()
#         else:
#             self.w.show()

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.window1 = AnotherWindow()
#         self.window2 = AnotherWindow()

#         l = QVBoxLayout()
#         button1 = QPushButton("Push for Window 1")
#         button1.clicked.connect(self.toggle_window1)
#         l.addWidget(button1)

#         button2 = QPushButton("Push for Window 2")
#         button2.clicked.connect(self.toggle_window2)
#         l.addWidget(button2)

#         w = QWidget()
#         w.setLayout(l)
#         self.setCentralWidget(w)

#     def toggle_window1(self, checked):
#         if self.window1.isVisible():
#             self.window1.hide()

#         else:
#             self.window1.show()

#     def toggle_window2(self, checked):
#         if self.window2.isVisible():
#             self.window2.hide()

#         else:
#             self.window2.show()

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.window1 = AnotherWindow()
#         self.window2 = AnotherWindow()

#         l = QVBoxLayout()
#         button1 = QPushButton("Push for Window 1")
#         button1.clicked.connect(
#             lambda checked: self.toggle_window(self.window1)
#         )
#         l.addWidget(button1)

#         button2 = QPushButton("Push for Window 2")
#         button2.clicked.connect(
#             lambda checked: self.toggle_window(self.window2)
#         )
#         l.addWidget(button2)

#         w = QWidget()
#         w.setLayout(l)
#         self.setCentralWidget(w)

#     def toggle_window(self, window):
#         if window.isVisible():
#             window.hide()

#         else:
#             window.show()
