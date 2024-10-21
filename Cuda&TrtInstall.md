following this cuda installation tutorial:

https://blog.csdn.net/weixin_37926734/article/details/123033286
https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#pre-installation-actions


1) install nvidia driver:
 if no additional driver in software and update, then go on nvidia to download runfile:
  https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html#runfile
  https://www.nvidia.com/Download/index.aspx?lang=en-us#
  
  may encounter this error during runfile installation:
	  One or more modprobe configuration files to disable Nouveau have been written.  For some distributions, this may be sufficient to disable Nouveau; other distributions may require modification of the initial ramdisk.  Please reboot your system and attempt NVIDIA driver   
	  installation again.  Note if you later wish to re-enable Nouveau, you will need to delete these files: /usr/lib/modprobe.d/nvidia-installer-disable-nouveau.conf, /etc/modprobe.d/nvidia-installer-disable-nouveau.conf

	if you are installing with runfile, it will fix it for you very likely
	
2) install CUDA
sudo apt-get install linux-headers-$(uname -r)
sudo apt-key del 7fa2af80
go on cuda toolkit download page, follow instruction there.
for reference, Simon's ALienware is using CUDA toolkit 11.8 and driver 520, its kernel version is 5.15.0-67-generic

3) install TensorRT
follow the Nvidia TensorRT installation guide, use tar option to offer more flexibility
download TRT tar package on Nvidia website
cd to where the download file is 
version="8.x.x.x"
arch=$(uname -m)
cuda="cuda-x.x"
tar -xzvf TensorRT-${version}.Linux.${arch}-gnu.${cuda}.tar.gz
put the TRT folder in home
sudo nano ~/.bashrc
paste this line at the end: export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/{user}/TensorRT-${version}/lib
cd TensorRT-${version}/python
python3 -m pip install tensorrt-*-cp3x-none-linux_x86_64.whl
cd TensorRT-${version}/graphsurgeon
python3 -m pip install graphsurgeon-0.4.6-py2.py3-none-any.whl
cd TensorRT-${version}/onnx_graphsurgeon
python3 -m pip install onnx_graphsurgeon-0.3.12-py2.py3-none-any.whl

---you should be done!