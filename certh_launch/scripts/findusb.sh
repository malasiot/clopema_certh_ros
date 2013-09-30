 #!/bin/bash 

if [ "$2" == "" ]; then
CMD=`lsusb -d "$1:"`
else
CMD=`ssh "$2" lsusb -d "$1:"`
fi

STR=`echo $CMD | sed -r 's/Bus ([0-9]+) Device ([0-9]+).*/\1 \2/'`
BUS=$(echo $STR | cut -f1 -d' ' | egrep -o '[1-9]+')
DEV=$(echo $STR | cut -f2 -d' ' | egrep -o '[1-9]+')
echo $BUS@$DEV
