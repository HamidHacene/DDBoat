git clone https://github.com/wjwwood/serial.git
cd serial
make -j2
make test
make docs
make install
cd ..

sudo apt install libgps-dev