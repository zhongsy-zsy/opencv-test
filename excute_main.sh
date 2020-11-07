# source 启动的脚本会影响当前终端
# sh执行的并不会影响当前终端
rm ~/depth_datas/*
cd ~/build_of_mvcompensation/realsense
./main
cd ..
