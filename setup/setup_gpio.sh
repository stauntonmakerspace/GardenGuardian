# Install the Jetson.GPIO library.
sudo apt install python3-pip
sudo pip3 install Jetson.GPIO

# Set up permissions to user and user groups.
sudo groupadd -f -r <gpio_group_name>
sudo usermod -a -G <gpio_group_name> <user_name>

sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

    