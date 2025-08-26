#!/usr/bin/env python
import rospy
import serial
import datetime
import time

def main():
    rospy.init_node("uart_trigger_node")

    port = rospy.get_param("~port", "/dev/ttyUSB2")
    baudrate = rospy.get_param("~baudrate", 115200)
    ser = serial.Serial(port, baudrate, timeout=1)

    on_cmd  = bytes([0x23, 0x01, 0x24])  # ON
    off_cmd = bytes([0x23, 0x00, 0x24])  # OFF

    rate = rospy.Rate(200)  # 200 Hz ~ 5ms resolution
    rospy.loginfo("UART Trigger Node started (ON ở giây chẵn, 0.2s)")

    last_sec = -1
    trigger_active = False
    trigger_start = None

    while not rospy.is_shutdown():
        now = datetime.datetime.now()
        sec = now.second
        now_time = now.timestamp()  # float (s.milliseconds)

        # Khi sang giây mới
        if sec != last_sec:
            last_sec = sec

            if sec % 2 == 0:  # giây chẵn
                ser.write(on_cmd)
                ser.flush()
                rospy.loginfo(f"[{now}] Sent ON (second={sec})")
                trigger_active = True
                trigger_start = now_time
            else:
                rospy.loginfo(f"[{now}] Odd second ({sec}), idle")


        rate.sleep()

    ser.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
