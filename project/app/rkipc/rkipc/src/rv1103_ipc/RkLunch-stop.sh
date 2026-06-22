#!/bin/sh

rcK()
{
	for i in $(ls /oem/usr/etc/init.d/S??*) ;do

		# Ignore dangling symlinks (if any).
		[ ! -f "$i" ] && continue

		case "$i" in
			*.sh)
				# Source shell script for speed.
				(
					trap - INT QUIT TSTP
					set stop
					. $i
				)
				;;
			*)
				# No sh extension, so fork subprocess.
				$i stop
				;;
		esac
	done
}

echo "Stop Application ..."
killall rkipc
killall udhcpc

# 等待 rkipc 退出，最多等 5 秒，超时则强制 kill -9
count=0
while [ $count -lt 5 ];
do
	sleep 1
	count=$((count + 1))
	ps|grep rkipc|grep -v grep
	if [ $? -ne 0 ]; then
		echo "rkipc exit"
		break
	else
		echo "rkipc active (${count}s)"
	fi
done

# 如果仍在运行，强制 kill -9
ps|grep rkipc|grep -v grep > /dev/null 2>&1
if [ $? -eq 0 ]; then
	echo "rkipc still running, force kill -9"
	killall -9 rkipc
	sleep 1
fi

rcK
