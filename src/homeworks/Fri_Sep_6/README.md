# 主任务：沿着线一直走
    分支1: 线没了，得找线
    分支2: 只要出现红灯，红灯立刻停；红灯撤回就继续走
    分支3: 有标记，需要瞄准标记，然后拍照保存。有1、2、3、4、5五张标记，有其他干扰。

    思路：一直开着摄像，5帧，然后每一帧都进行判断。
    """
    status=0: go(), find_light(), find_marker()
    status=1:          go() -> deal_line()
    status=2:  find_light() -> deal_light()
    status=3: find_marker() -> deal_marker()
    
    at any time: get_image()
    
    go():          status: 0 -> 1
    find_light():  status: 0 -> 2
    find_marker(): status: 0 -> 3
    
    deal_line():   status: 1 -> 0
    deal_light():  status: 2 -> 0
    deal_marker(): status: 3 -> 0
    
    status: int
    """

## Issues:
    1. 使用队友的find_line()
    2. PID参数调节
    3. 识别marker函数

## Fixed:
    1. 连续好多次not_find才进入deal_line()
    2. 使用time.sleep()减轻了marker和line回调问题

PS：性能严重不足
