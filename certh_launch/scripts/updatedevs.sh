 #!/bin/bash 

infile=`rospack find certh_launch`/config/devices.cfg
scr="`rospack find certh_launch`/scripts/findusb.sh"
outfile=`rospack find certh_launch`/launch/devices.launch 

echo '<?xml version="1.0"?>' > $outfile
echo '<launch>' >> $outfile 

IFS=$'\n'
for line in $(cat $infile)
do
IFS=$'\t' read -a tokens <<< "$line"
param=${tokens[0]} 
desc=${tokens[1]}
remote=${tokens[2]}
res=`$scr "$desc" "$remote"`
echo '<param name="'$param'" value="'$res'"/>' >> $outfile
done

echo '</launch>' >> $outfile
