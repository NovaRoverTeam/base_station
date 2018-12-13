#!/usr/bin/env python

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# File: gui_vis.py
# Author: Ben Steer
# Last modified by: Ben Steer
#
# Description:
#   This class contains methods for changing visual elements of the
#   GUI. The purpose of this class is to separate the programmatic
#   visual changes in gui_backend.py from its functional features. Do
#   not introduce ROS elements to this code!
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

from PyQt4 import QtCore, QtGui # Qt includes
from PyKDE4.kdeui import *

# Class representing the visual GUI interface
class GuiVis():

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():    #
    #    Initiases main class.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
    def __init__(self, ui):

        self.ui = ui  # Qt MainDialog (for connecting ROS msg callbacks)

        self.leds = [
            ui.led_GPS, ui.led_LIDAR, ui.led_Raman, ui.led_haptic, 
            ui.led_xbox, ui.led_IMU, ui.led_rosbag, ui.led_battery, 
            ui.led_relay
        ]
        self.led_cams = [
            ui.led_cam0, ui.led_cam1, ui.led_cam2, ui.led_cam3, 
            ui.led_cam4
        ]
        self.cam_panes = [
            ui.cam0_pane, ui.cam1_pane, ui.cam2_pane, ui.cam3_pane,
            ui.cam4_pane
        ]
        self.tool_cam_shows = [
            ui.tool_cam0_show, ui.tool_cam1_show, ui.tool_cam2_show,
            ui.tool_cam3_show, ui.tool_cam4_show 
        ]
        self.tool_cam_starts = [
            ui.tool_cam0_start, ui.tool_cam1_start, ui.tool_cam2_start,
            ui.tool_cam3_start, ui.tool_cam4_start
        ]
        self.mode_buttons = [
            ui.button_standby, ui.button_drive, ui.button_arm,
            ui.button_drill, ui.button_auto
        ]        
        self.sliders = [
            ui.slider_a, ui.slider_b, ui.slider_c
        ]
        self.tool_sliders = [
            ui.tool_slider_a, ui.tool_slider_b, ui.tool_slider_c
        ]
        self.vehicle_buttons = [
            ui.button_simulator, ui.button_rover, ui.button_sandstorm
        ]
        self.progress_radios = [
            ui.progress_radio0, ui.progress_radio1
        ]
        self.led_radio_plugged = [
            ui.led_radio0_plugged, ui.led_radio1_plugged
        ]
        self.led_radio_paired = [
            ui.led_radio0_paired, ui.led_radio1_paired
        ]
        self.radio_debug_buttons = [
            ui.tool_900_debug, ui.tool_5_debug
        ]

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # initialiseWidgets(): 
    #   Setup widget initial values.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def initialiseWidgets(self, rpm_limit, steer_limit): 

        self.ui.combo_mission.setCurrentIndex(0) # Switch to setup pane
        self.ui.stack_mission.setCurrentIndex(0) 
        
        # TODO fix
        self.ui.slider_a.setProperty("value", round(rpm_limit*100))
        self.ui.slider_b.setProperty("value", round(steer_limit*100))  
        
        # Restrict lat/lng fields to only accept doubles
        self.ui.edit_lat.setValidator(QtGui.QDoubleValidator())                       
        self.ui.edit_lng.setValidator(QtGui.QDoubleValidator())    

        mode_stylesheet = "QPushButton:checked { background-color: orange; }\n"
        for mode_button in self.mode_buttons:
            mode_button.setStyleSheet(mode_stylesheet)

        for led in (self.leds + self.led_cams):
            led.off()

        for cam_pane in self.cam_panes:
            cam_pane.setEnabled(False)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # setupWidgetConnections(): 
    #   Setup widget connections for signals.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def setupWidgetConnections(self): 

        for tool_cam_show in self.tool_cam_shows:
            tool_cam_show.clicked.connect(self.ui.toggleCamView)

        for tool_cam_start in self.tool_cam_starts:
            tool_cam_start.clicked.connect(self.ui.toggleStream)

        self.ui.button_simulator.clicked.connect(self.ui.launchSimulator)
        self.ui.button_engage_auto.clicked.connect(self.ui.engageAuto)
        
        for mode_button in self.mode_buttons:
            mode_button.clicked.connect(self.ui.modeChange)    
        
        for slider in self.sliders:
            slider.valueChanged.connect(self.ui.sliderChange)

        for tool_slider in self.tool_sliders:
            tool_slider.toggled.connect(self.ui.sliderUnlock) 

        self.ui.checkbox_radio_debug.toggled.connect(self.ui.toggleRadioDebug)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # clearModeButtons(): 
    #   Uncheck all the mode buttons.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def clearModeButtons(self): 

        for mode_button in self.mode_buttons:
            mode_button.setChecked(False)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # enableModeButtons(): 
    #   Enable or disable all the mode buttons.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def enableModeButtons(self, enable): 

        for mode_button in self.mode_buttons:
            mode_button.setEnabled(enable)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # enableRadioDebug(): 
    #   Enable or disable the radio debug buttons
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def enableRadioDebug(self, enable): 

        for radio_debug_button in self.radio_debug_buttons:
            radio_debug_button.setEnabled(enable)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # enableCamPanes(): 
    #   Enable or disable all the camera panes.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def enableCamPanes(self, enable): 
        
        for cam_pane in self.cam_panes:
            cam_pane.setEnabled(enable)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # enableVehicleButtons(): 
    #   Enable or disable all the vehicle buttons.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def enableVehicleButtons(self, enable): 

        for vehicle_button in self.vehicle_buttons:
            vehicle_button.setEnabled(enable)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # getCamWidgets(): 
    #   Retrieve LED, show and start buttons for cameras.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def getCamWidgets(self, index): 

        led           = self.led_cams[index]
        button_start  = self.tool_cam_starts[index]
        button_show   = self.tool_cam_shows[index]

        return led, button_start, button_show

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # getRadioWidgets(): 
    #   Retrieve LEDs and progress bar for radios.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def getRadioWidgets(self, index): 

        progress = self.progress_radios[index]
        plugged  = self.led_radio_plugged[index]
        paired   = self.led_radio_paired[index]

        return progress, plugged, paired
