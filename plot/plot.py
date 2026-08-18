import lcm
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
import numpy as np

from plot_msg import plot  # 导入你的LCM消息定义

# ========== 配置参数 ==========
max_points = 100          # 每条轨迹保留的最大点数
# 4个足端对应的LCM通道名（请根据你实际的通道名修改）
leg_channels = [
    "plot_rf",   # 右前足
    "plot_lf",   # 左前足
    "plot_rr",   # 右后足
    "plot_lr"    # 左后足
]
# 4条轨迹的颜色 (R, G, B, A)
leg_colors = [
    (1, 0, 0, 1),    # 红 - 右前
    (0, 1, 0, 1),    # 绿 - 左前
    (0, 0, 1, 1),    # 蓝 - 右后
    (1, 1, 0, 1)     # 黄 - 左后
]
leg_names = ["右前", "左前", "右后", "左后"]

# ========== 全局数据存储 ==========
# 存储每条腿的历史轨迹点
points_list = [np.zeros((0, 3)) for _ in range(4)]
# 存储每条腿的最新坐标
current_pos = [[0.0, 0.0, 0.0] for _ in range(4)]


def make_handler(leg_index):
    """生成对应腿的LCM回调函数，闭包保存腿索引"""
    def handler(channel, data):
        msg = plot.decode(data)
        current_pos[leg_index][0] = msg.x
        current_pos[leg_index][1] = msg.y
        current_pos[leg_index][2] = msg.z
    return handler


def update_plot():
    """更新所有4条轨迹"""
    for i in range(4):
        # 添加新点
        new_point = np.array([[current_pos[i][0], current_pos[i][1], current_pos[i][2]]])
        points_list[i] = np.vstack([points_list[i], new_point])
        
        # 限制点数量
        if len(points_list[i]) > max_points:
            points_list[i] = points_list[i][1:]
        
        # 更新轨迹线和当前点
        trajectory_lines[i].setData(pos=points_list[i])
        current_points[i].setData(pos=new_point)


def handle_lcm():
    """处理LCM消息，超时1ms避免阻塞UI"""
    lc.handle_timeout(1)


if __name__ == '__main__':
    # 初始化LCM，订阅4个通道
    lc = lcm.LCM()
    for idx, channel in enumerate(leg_channels):
        lc.subscribe(channel, make_handler(idx))

    # 初始化Qt和3D视图
    app = pg.mkQApp("四足轨迹")
    view = gl.GLViewWidget()
    view.show()
    view.setWindowTitle('四足足端3D轨迹')
    # 相机拉远一点，保证四条腿都在视野内
    view.setCameraPosition(distance=0.8, elevation=30, azimuth=45)

    # 添加坐标轴网格
    grid = gl.GLGridItem()
    grid.scale(0.1, 0.1, 0.1)  # 网格大小0.1m
    view.addItem(grid)

    # 批量初始化4条轨迹线和4个当前点
    trajectory_lines = []
    current_points = []
    for i in range(4):
        line = gl.GLLinePlotItem(color=leg_colors[i], width=2)
        point = gl.GLScatterPlotItem(color=leg_colors[i], size=10)
        view.addItem(line)
        view.addItem(point)
        trajectory_lines.append(line)
        current_points.append(point)

    # LCM消息处理定时器（10ms）
    lcm_timer = QtCore.QTimer()
    lcm_timer.timeout.connect(handle_lcm)
    lcm_timer.start(10)

    # 画面刷新定时器（30ms）
    plot_timer = QtCore.QTimer()
    plot_timer.timeout.connect(update_plot)
    plot_timer.start(30)

    pg.exec()
