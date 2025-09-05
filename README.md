# hil-based-rail-manipulation
readme具体参考examples/experiments/pick_ironroad下的readme。

我们的部署代码主要写在examples里面，examples*会不断迭代，可以补充自己的内容发布或者建立新的分支。

examples3修复了ros和3.8的依赖问题。
注意事项：
使用本项目conda环境编译arm_contral，添加以下两个命令到~/.bashrc
export PYTHONPATH="/home/wang/Project/hil-serl1/serl_launcher:/home/wang/Project/hil-serl1/examples2:$PYTHONPATH"
source /home/wang/Project/hil-serl1/arm_control/teleop/install/setup.bash

下一步使用真实的模型检查单测试
