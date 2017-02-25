#!/bin/bash
echo "Shutting down driver!"
sudo modprobe -r pn_console_controller
echo "Removing old driver!"
sudo dkms remove -m pn_console_controller -v 0.1.0 --all
echo "Copying files!"
sudo cp -a * /usr/src/pn_console_controller-0.1.0/
echo "Building driver!"
sudo dkms build -m pn_console_controller -v 0.1.0
echo "Installing driver!"
sudo dkms install -m pn_console_controller -v 0.1.0
echo "Done!"

