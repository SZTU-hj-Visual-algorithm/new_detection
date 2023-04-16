#!/bin/bash

sec=1
cnt=0
name=hj_v
cd /home/nvidia/Desktop/hjomni/build/
make -j

while true; do

    p=$(pidof $name)
    echo "now_PID is $p"

    if pidof $name > /dev/null; then
        echo "$name is still alive!"
        sleep $sec

        ppp=$(ps -p $(pidof $name) -o %cpu | awk 'NR==2 {printf("%.0f", $1)}')
        echo "now CPU use $ppp"

        if [ $ppp -lt 5 ]; then
            echo "enter"
            echo "Starting $name..."
            kill $p
            cd /home/nvidia/Desktop/hjomni/build/
            ./$name &
            echo "$name has started!"
            cnt=`expr $cnt + 1`
            echo "cnt: $cnt"
            sleep $sec

        fi


    else
        echo "Starting $name..."
        cd /home/nvidia/Desktop/hjomni/build/
        ./$name &
        echo "$name has started!"
        cnt=`expr $cnt + 1`
        echo "cnt: $cnt"
        sleep $sec
    fi
done
