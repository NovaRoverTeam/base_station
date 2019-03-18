# base_station
Provides interfaces for rover management and monitoring on the Linux base station computer.

# Dependencies

ROS uses python 2.7 so if pip installs these packages in python 3, you might have to use pip2 install instead.

- Python modules:
```
pip install transitions
pip install pexpect
```

- Qt and KDE for GUI:
```
sudo apt install python-qt4
sudo apt install pyqt4-dev-tools
sudo apt install python-kde4
```

Probs won't need all these, but why not install them anyways
- GStreamer for Python:
```
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-pulseaudio
```
