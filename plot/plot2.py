import lcm
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
import numpy as np

from plot_msg import plot  # 导入您的LCM消息定义

max_points = 100
points = np.zeros((0, 3))
x = y = z = 0.0

def handler(channel, data):
    global x, y, z
    msg = plot.decode(data)
    x = msg.x
    y = msg.y
    z = msg.z
    # print(f"收到: x={msg.x:.5f}, y={msg.y:.5f}, z={msg.z:.5f}")


def update_plot():
    global points
        
    # 添加新点
    new_point = np.array([[x, y, z]])
    points = np.vstack([points, new_point])
    
    # 限制点数量
    if len(points) > max_points:
        points = points[1:]
    
    # 更新显示
    trajectory_line.setData(pos=points)
    current_point.setData(pos=new_point)

def handle_lcm():
    # 超时时间1ms，避免阻塞Qt事件循环
    lc.handle_timeout(1)

if __name__ == '__main__':
    lc = lcm.LCM()
    lc.subscribe("plot2", handler)
    app = pg.mkQApp("足端轨迹")
    view = gl.GLViewWidget()
    view.show()
    view.setWindowTitle('右前足端3D轨迹')
    view.setCameraPosition(distance=0.5, elevation=30, azimuth=45)

    # 添加坐标轴网格
    grid = gl.GLGridItem()
    grid.scale(0.1, 0.1, 0.1)  # 网格大小0.1m
    view.addItem(grid)

    # 初始化轨迹线和当前点
    trajectory_line = gl.GLLinePlotItem(color=(1, 0, 0, 1), width=2)
    current_point = gl.GLScatterPlotItem(color=(0, 1, 0, 1), size=10)
    view.addItem(trajectory_line)
    view.addItem(current_point)

    lcm_timer = QtCore.QTimer()
    lcm_timer.timeout.connect(handle_lcm)
    lcm_timer.start(10)

    plot_timer = QtCore.QTimer()
    plot_timer.timeout.connect(update_plot)
    plot_timer.start(30)


    pg.exec()
