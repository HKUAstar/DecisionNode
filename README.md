# DecisionNode
The decision part for sentry using behaviour tree

手动发布数据进行测试：

run test.cpp:/mnt/d/decision_ws/devel/lib/decision_node/continuous_forwarder 

可视化：下载Groot2，打开xml文件自动识别

# 节点介绍，以免写到后面忘记什么写过什么没写过了

数据更新节点

    <UpdateNavigationBB />

    <UpdateVisionBB />

    <UpdateTimersBB />
    
    <UpdateDerivedFlags danger_hp="{danger_hp}" sufficient_bullet="{sufficient_bullet}" />

Condition节点

    <IsGameStarted expect_started="false" /> false为比赛未开始时返回success; true为比赛开始返回success

    <IsAction value="INIT" />判断当前状态是否是某个指定的值

    <IsSentryDead>
    <IsSentryAlive />

    <IsSentryInDanger />低于指定血量返回success

    <NotBulletSufficient />子弹量小于阈值返回success

    

SetBulletNum (DELTA是补弹到某个固定值/FIXED 计算)

<Wait duration="2.0" />--等待指定秒数