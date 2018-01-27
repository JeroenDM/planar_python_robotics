from PyQt5 import QtWidgets
import sys

# Local Module Imports
import app_framework as af

# Create GUI application
app = QtWidgets.QApplication(sys.argv)
form = af.MyApp()
form.show()
app.exec_()
