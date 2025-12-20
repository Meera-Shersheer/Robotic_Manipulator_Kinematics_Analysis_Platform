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
