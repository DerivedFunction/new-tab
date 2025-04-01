# Clean install and transfers files to capstone-nx
rm -rf mdc
tar -xvf mdc.gz
cd mdc
chmod +x init.sh
./init.sh
./transfer.sh send ~/Downloads/mdc.gz .
./xbox.sh

