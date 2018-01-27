from PyQt5.QtWidgets import QApplication, QMainWindow
import design

class MyApp(QMainWindow, design.Ui_MainWindow):
    def __init(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.pushButton_plotData.clicked.connect(self.plot_data)

    def plot_data(self):
        x=range(0, 10)
        y=range(0, 20, 2)
        self.plotWidget.canvas.ax.plot(x, y)
        self.plotWidget.canvas.draw()
