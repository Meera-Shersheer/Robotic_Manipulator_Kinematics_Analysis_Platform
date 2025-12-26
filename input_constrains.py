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
    
#    """Delegate to handle both numeric and symbolic input in matrix cells"""
class MatrixDelegate(QStyledItemDelegate):
    def __init__(self, parent, get_computation_mode_callback):
        super().__init__(parent)
        self.get_computation_mode = get_computation_mode_callback
    
    def createEditor(self, parent, option, index):
        editor = QLineEdit(parent)
        editor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Add validator only for numeric mode
        if self.get_computation_mode() == 1:  # 0 = numeric
            editor.setValidator(QDoubleValidator())
        # For symbolic mode (1), no validator - accept any text
        
        return editor
    
    def setEditorData(self, editor, index):
        value = index.model().data(index, Qt.ItemDataRole.EditRole)
        editor.setText(str(value) if value else "0")
    
    def setModelData(self, editor, model, index):
        text = editor.text().strip()
        if not text:
            text = "0"
        model.setData(index, text, Qt.ItemDataRole.EditRole)