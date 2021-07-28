# Install - Ubuntu 20.04
This guide assumes a fresh install of Ubuntu 20.04. Other linux distros may function properly, but are unsupported.

1. Ensure your operating system is up to date:
   1. In the terminal, run the following: `sudo apt update && sudo apt upgrade -y`
1. Dowload & install the latest Nvidia drivers:
   1. Check your device's drivers with the command: `ubuntu-drivers devices`
   2. Select the recommended driver for your hardware using `sudo apt install DRIVER`. For example, `sudo apt install nvidia-driver-470`.
