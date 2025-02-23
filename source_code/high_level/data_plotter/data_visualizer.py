import sys
import time
import csv
from collections import deque

import pyqtgraph as pg
# Import QtCore, QtGui, and QtWidgets from pyqtgraph.Qt, which detects PyQt6
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

CSV_FILENAME = "log.csv"
UPDATE_INTERVAL_SEC = 0.01
MAX_SAMPLES = 1000

class RealTimePlotter:
    def __init__(self):
        # With PyQt6, QApplications are in QtWidgets
        self.app = QtWidgets.QApplication(sys.argv)

        self.data1 = deque(maxlen=MAX_SAMPLES)
        self.data2 = deque(maxlen=MAX_SAMPLES)
        self.data3 = deque(maxlen=MAX_SAMPLES)
        self.data4 = deque(maxlen=MAX_SAMPLES)
        self.last_line_count = 0

        # Create the PyQtGraph window
        self.win = pg.GraphicsLayoutWidget(title="Real-Time CSV Plotter")
        self.win.show()

        # Add a single PlotItem
        self.plot = self.win.addPlot(title="Data from CSV")
        self.plot.showGrid(x=True, y=True)
        self.plot.addLegend(offset=(-50,10))

        # Create four curves for data1..data4
        self.curve1 = self.plot.plot(pen='r', name="data1")
        self.curve2 = self.plot.plot(pen='g', name="data2")
        self.curve3 = self.plot.plot(pen='b', name="data3")
        self.curve4 = self.plot.plot(pen='y', name="data4")

        # Timer to periodically update the plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updatePlot)
        self.timer.start(int(UPDATE_INTERVAL_SEC * 1000))

    def readNewData(self):
        """
        Reads new lines from the CSV file and appends them to the data queues.
        """
        try:
            with open(CSV_FILENAME, 'r') as f:
                reader = csv.reader(f)
                all_lines = list(reader)
                current_line_count = len(all_lines)

                if current_line_count > self.last_line_count:
                    new_lines = all_lines[self.last_line_count:current_line_count]

                    for row in new_lines:
                        # Expect 5 columns: [timestamp, data1, data2, data3, data4]
                        if len(row) < 5:
                            continue
                        try:
                            val1 = float(row[1])
                            val2 = float(row[2])
                            val3 = float(row[3])
                            val4 = float(row[4])
                            self.data1.append(val1)
                            self.data2.append(val2)
                            self.data3.append(val3)
                            self.data4.append(val4)
                        except ValueError:
                            # Skip lines that aren't purely numeric in the data columns
                            continue

                    self.last_line_count = current_line_count

        except FileNotFoundError:
            # If the file doesn't exist, silently ignore (until next update)
            pass

    def updatePlot(self):
        """
        Periodically called by the timer to read new CSV data and update the plot curves.
        """
        self.readNewData()
        x_range = range(len(self.data1))

        self.curve1.setData(x=list(x_range), y=list(self.data1))
        self.curve2.setData(x=list(x_range), y=list(self.data2))
        self.curve3.setData(x=list(x_range), y=list(self.data3))
        self.curve4.setData(x=list(x_range), y=list(self.data4))

    def run(self):
        sys.exit(self.app.exec())  # Note: PyQt6 uses 'exec()' instead of 'exec_()'

if __name__ == '__main__':
    plotter = RealTimePlotter()
    plotter.run()
