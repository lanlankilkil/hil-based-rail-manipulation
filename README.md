# hil-based-rail-manipulation
1.readme具体参考examples/experiments/pick_ironroad下的readme。

2.examples3修复了ros和python版本的代码不兼容问题。目前可以在foxy正常运行。

注意事项：使用本项目conda环境编译arm_contral，添加以下两个命令到~/.bashrc

export PYTHONPATH="/home/your_name/Project/hil-serl1/serl_launcher:/home/wang/Project/hil-serl1/examples*:$PYTHONPATH"

source /your_name/wang/Project/hil-serl1/arm_control/teleop/install/setup.bash
下一步使用真实的模型检查单测试
