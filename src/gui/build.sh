#!/bin/bash
 
pyuic4 main.ui -o main.py

cd resources
pyrcc4 -o res_rc.py res.qrc
cp res_rc.py ../res_rc.py
rm res_rc.py

echo "Finished rebuilding GUI."
