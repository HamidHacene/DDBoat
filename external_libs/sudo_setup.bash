source /opt/ros/kinetic/setup.bash
unset https_proxy
unset http_proxy

cp -R /tmp/usr/local/include/* /usr/local/include/
cp -R /tmp/usr/local/lib/* /usr/local/lib/
apt install libgps-dev
ldconfig 