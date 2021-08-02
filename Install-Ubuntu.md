# Install - Ubuntu 20.04
This guide assumes a fresh install of Ubuntu 20.04. Some software conflicts with OpenPose, such as Anaconda - check the [OpenPose Documentation](https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_00_index.html). Other linux distros may function properly, but are unsupported.

1. Ensure your operating system is up to date:
   1. In the terminal, run the following: `sudo apt update && sudo apt upgrade -y`
2. Dowload & install the latest Nvidia drivers:
   1. Check your device's drivers with the command: `ubuntu-drivers devices`
   2. Select the recommended driver for your hardware using `sudo apt install DRIVER-NAME`. For example, `sudo apt install nvidia-driver-470`.
   3. Restart your computer.
4. Install latest CMake GUI:
   1. Run `sudo apt-get install build-essential` and `sudo apt-get install libssl-dev` to get prerequisites.
   2. Purge any existing CMake-gui install using `sudo apt purge cmake-qt-gui`.
   3. Run `sudo apt-get install qtbase5-dev` for use in building CMake.
   4. Run `sudo apt-get update && sudo apt-get upgrade` to ensure everything is up to date.
   5. Go to https://cmake.org/download/ and download the latest **Unix Source distribution** tar file.
   6. Navigate to the download location using the terminal.
   7. Extract the tar file using `tar -zxvf FILENAME` and enter the newly created directory.
   8. Run `./configure --qt-gui` (Make sure you are in the right directory!).
   9. Run `./bootstrap` (Note: steps ix-xi take a while, and can be combined with `&&` between each command so user input is not required).
   10. Run ```` make -j`nproc` ````
   11. Run ```` sudo make install -j`nproc` ````
   12. Make sure CMake is installed and is the correct version by checking Ubuntu's application list in the menu bar ![Screenshot](https://user-images.githubusercontent.com/70712042/127895411-07ad3984-3c8c-4c43-acb8-d67bec292ae2.png)
5. Install Nvidia CUDA & cuDNN:
   1. Go [here](https://developer.nvidia.com/cuda-11.1.1-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=2004) to install version 11.1.1 of CUDA for Ubuntu 20.04. **ONLY RUN THE WGET COMMAND FOR NOW:** `wget https://developer.download.nvidia.com/compute/cuda/11.1.1/local_installers/cuda_11.1.1_455.32.00_linux.run`
   2. Follow the [CUDA 11.1.1 runfile installation instructions](https://docs.nvidia.com/cuda/archive/11.1.1/cuda-installation-guide-linux/index.html#runfile) **carefully**. Follow every step, and make sure you know what each step is doing! It is strongly reccomended you open this guide on a separate device. Some help for particular steps are offered:
      1. Uninstall both the Nvidia Drivers **AND** blacklist the Nouveau drivers. To uninstall all Nvidia-related items, use `sudo apt-get remove --purge '^nvidia-.*'` and then `sudo apt-get autoremove`. **MAKE SURE YOU KNOW WHAT TO DO WHEN REBOOTING YOUR COMPUTER, SINCE YOU WON'T HAVE A DRIVER UNTIL CUDA IS INSTALLED**
      2. To blackist Nouveau, use the following commands: run `sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"`, run `sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"`, and update your kernel with `sudo update-initramfs -u`. To reverse this in case of a failed installation, use `sudo rm -rf /etc/modprobe.d/blacklist-nvidia-nouveau.conf` and update your kernel again.
      3. Instead of manually editing your kernel to enter text mode, select **Advanced Options>Recovery Mode** in the grub boot-loader menu on startup. Then, select **Drop to root shell prompt** to launch in text mode.
      4. When running the .sh file to install CUDA, make sure that it does not say anything about existing packages. If it does, use `dpkg -l | grep nvidia` and `sudo apt purge name-of-package` until the dpkg command returns nothing.
      5. Use `sudo reboot` to restart your computer after the installation is complete, and boot normally. Assuming the installation is complete, the desktop should be visible and look normal. If this is not the case, something went wrong and recovery mode should be used to restore the Nouveau drivers so that Ubuntu can start properly.
      6. Ensure the installation was successful by checking Sotware & Updates > Additional Drivers and check for "Continue using a manually installed driver", and that a CUDA toolkit is located in /usr/local/.
   3. Download cuDNN from [here](https://developer.nvidia.com/cudnn). Make sure to install cuDNN v8.0.4 for CUDA 11.1, Linux x86_64.
   4. Install by extracting the downloaded file to /usr/local/cuda. Detailed instructions can be found [here](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux-tar).
6. Install OpenCV and the camera calibration app.
   1. Install prerequisites using `sudo apt install libgtk-3-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
gfortran openexr libatlas-base-dev python3-dev python3-numpy \
libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev`.
   2. Download OpenCV by moving to a suitable directory and using `git clone https://github.com/opencv/opencv.git` and `git -C opencv checkout master`.
   3. Enter the newly created directory and use `mkdir -p build && cd build` to create the build directory.
   4. Use `cmake ..` and ```` make -j`nproc` ```` to configure and build OpenCV.
   5. Use `sudo make install` for a system-wide installation.
   6. Download the CameraCalibration folder, consisting of a .cpp file and a MakeFile.
   7. Enter its directory, and use `cmake .` and `make` to configure and build the Camera Calibration app.
   8. Launch the application using `./CameraCalibCV`. You may calibrate your cameras now if you wish, but make sure the application functions properly before continuing.
7. Install OpenPose
   1. Download OpenPose using `git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose`.
   3. Install prerequisites using `sudo bash ./scripts/ubuntu/install_deps.sh`.
   4. Use `mkdir build && cd ./build` to create and enter the build directory.
   5. Use `cmake-gui ..` and select **Configure**. Wait until new options are shown highlighted in red, and select your preferences. Then, select **Configure** and **Generate** and close the GUI.
   6. Build OpenPose using ```` make -j`nproc` ````.
