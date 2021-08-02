# Install - Ubuntu 20.04
This guide assumes a fresh install of Ubuntu 20.04. Some software conflicts with OpenPose, such as Anaconda - check the [OpenPose Documentation](https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_00_index.html). Other linux distros may function properly, but are unsupported.

1. Ensure your operating system is up to date:
   1. In the terminal, run the following: `sudo apt update && sudo apt upgrade -y`
2. Dowload & install the latest Nvidia drivers:
   1. Check your device's drivers with the command: `ubuntu-drivers devices`
   2. Select the recommended driver for your hardware using `sudo apt install DRIVER-NAME`. For example, `sudo apt install nvidia-driver-470`.
   3. Restart your computer.
3. Install latest CMake GUI:
   1. Go to https://cmake.org/download/ and download the latest **Linux x86_64 Binary distribution** tar file.
   2. Navigate to the download location using the terminal.
   3. Extract the .tar file using `tar -zxvf FILENAME.tar` and enter the newly created cmake folder.
   4. Run `sudo apt-get install build-essential` to get prerequisites such as the g++ compiler.
   5. Run `sudo ./bootstrap` (Make sure you are in the cmake directory!).
   6. 
