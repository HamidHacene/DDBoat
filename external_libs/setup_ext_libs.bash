
source /opt/ros/kinetic/setup.bash
unset https_proxy
unset http_proxy

git clone https://github.com/wjwwood/serial.git
cd serial
make -j2
make test
make docs
make install
cd ..
