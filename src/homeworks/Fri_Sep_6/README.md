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
    1. 清空文件夹(小问题
    2. 

## Fixed:
    1.  连续好多次not_find才进入deal_line()
    2.  使用time.sleep()减轻了marker和line回调问题
    3.  不用队友的find_line()了，不好用
    4.  拍照成功
    5.  拍照不在正中心、重复拍照
    6.  PID参数调节
    7.  近距离才拍照，远距离不拍，改用另一个sub函数
    8.  图片上打印小组名
    9.  断线循线 -> 底盘跟随云盘
    10. 修复了转动云台时会卡住的问题
    11. 识别marker函数时出现了 list index out of range
    12. 图片上将marker标记
    13. 解决了多图像识别问题
    14. 解决了拍照射击问题
    15. 

PS：完全够用，死机就是程序问题
