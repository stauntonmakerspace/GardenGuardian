# Install the Jetson.GPIO library.
sudo apt install python3-pip
sudo pip3 install Jetson.GPIO

# Set up permissions to user and user groups.
sudo groupadd -f -r gpio
sudo usermod -a -G gpio rc

sudo cp venv/lib/pythonNN/site-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules && sudo udevadm trigger

    