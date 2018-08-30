#!/usr/bin/env python

from PyQt4 import QtCore, QtGui
import sys
from main import Ui_MainWindow

def rosbag_start(toggled):
  if toggled:
    print("gon rosbag now")
  else:
    print("dun rosbaggin")


def main():
  app = QtGui.QApplication(sys.argv)
  window = QtGui.QMainWindow()
  
  ui = Ui_MainWindow()
  ui.setupUi(window)
  
  ui.tool_rosbag_start.toggled.connect(rosbag_start)
  ui.progress_radio0.setProperty("value", -45)
  
  # 180 deg is North, positive increase is clockwise
  bearing = 135
  ui.dial_bearing.setProperty("value", 180 + bearing)
  ui.label_bearing.setText(str(bearing) + " deg")
  
  voltage = 23.5
  ui.label_voltage.setText(str(voltage) + " V")
  ui.progress_voltage.setProperty("minimum", 230)
  ui.progress_voltage.setProperty("maximum", 252)
  ui.progress_voltage.setProperty("value", round(voltage*10))
  
  window.show()  
  sys.exit(app.exec_())
  

if __name__ == '__main__':
  main()
