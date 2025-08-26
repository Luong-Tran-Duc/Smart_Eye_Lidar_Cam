#!/bin/bash
sudo ptpd -M -i eth2
sudo chmod 777 /dev/tty*
echo rs485 | sudo tee /sys/class/sp339_mode_ctl/uartMode > /dev/null
roslaunch common all.launch
