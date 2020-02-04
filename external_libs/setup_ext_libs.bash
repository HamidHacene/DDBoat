cd serial
make -j2
make test
make docs
make install
cd ..

sudo apt install libgps-dev