#!/bin/bash

SCRIPT_NAME="capture_image.py"
PID_FILE="capture_image.pid"

start() {
    /usr/bin/python3 $SCRIPT_NAME &
    echo $! > $PID_FILE
    echo "Script started with PID $!"
}

stop() {
    if [ -f $PID_FILE ]; then
        kill $(cat $PID_FILE)
        rm $PID_FILE
        echo "Script stopped"
    else
        echo "Script is not running"
    fi
}

case "$1" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    *)
        echo "Usage: $0 {start|stop}"
        exit 1
esac
