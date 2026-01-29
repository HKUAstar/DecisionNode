# DecisionNode
The decision part for sentry using behaviour tree

手动发布数据进行测试：

run test.cpp:/mnt/d/decision_ws/devel/lib/decision_node/continuous_forwarder 

可视化：下载Groot2，打开xml文件自动识别

# 节点介绍

数据更新节点

    <UpdateNavigationBB />

    <UpdateVisionBB />

    <UpdateTimersBB />
    
    <UpdateDerivedFlags danger_hp="{danger_hp}" sufficient_bullet="{sufficient_bullet}" />

<Wait duration="2.0" />--等待指定秒数