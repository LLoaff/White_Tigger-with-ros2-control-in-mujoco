#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import lcm
import curses
import threading
import time
import sys
from lcm_msg import lcm_vel_cmd  # 导入您的LCM消息定义

class LCMTeleopKeyboard:
    def __init__(self):
        # LCM初始化
        self.lcm = lcm.LCM()
        self.msg = lcm_vel_cmd()
        
        # 初始状态
        self.msg.vx = 0.0
        self.msg.vy = 0.0
        self.msg.vw = 0.0
        self.msg.state = 0
        self.msg.turn = 0
        
        # 速度步长和限制（新增vy相关参数）
        self.vx_step = 0.1    # 每次w/s调整的vx增量
        self.vy_step = 0.1    # 每次z/c调整的vy增量
        self.vw_step = 0.1    # 每次a/d调整的vw增量
        self.vx_max = 1.0      # vx最大限制
        self.vy_max = 1.0      # vy最大限制
        self.vw_max = 2.0      # vw最大限制
        
        # 发布频率 (Hz)
        self.publish_rate = 10.0
        
        # 线程控制
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        
        # 更新帮助信息，添加z/c控制说明
        self.help_msg = """
=============================================
          LCM 键盘控制程序
=============================================
控制说明:
  0-9 数字键: 设置 state 值 (0-9)
  w/s:       增加/减少 vx (前后速度)
  z/c:       增加/减少 vy (左右速度)
  a/d:       增加/减少 vw (转向速度)
  空格:      清除所有速度 (紧急停止)
  q:         退出程序
=============================================
当前状态:
  state: {state}
  vx:    {vx:.2f} m/s
  vy:    {vy:.2f} m/s
  vw:    {vw:.2f} rad/s
=============================================
"""

    def publish_loop(self):
        rate = 1.0 / self.publish_rate
        while self.running:
            try:
                self.lcm.publish("cmd_vel", self.msg.encode())
            except Exception as e:
                print(f"发布LCM消息失败: {e}")
            time.sleep(rate)

    def clamp_speed(self):
        """限制所有速度在安全范围内（新增vy限制）"""
        self.msg.vx = max(-self.vx_max, min(self.vx_max, self.msg.vx))
        self.msg.vy = max(-self.vy_max, min(self.vy_max, self.msg.vy))
        self.msg.vw = max(-self.vw_max, min(self.vw_max, self.msg.vw))

    def clear_speed(self):
        """清除所有速度"""
        self.msg.vx = 0.0
        self.msg.vy = 0.0
        self.msg.vw = 0.0

    def run(self, stdscr):
        curses.noecho()          # 关闭按键回显
        curses.cbreak()          # 无需回车立即响应
        stdscr.keypad(True)      # 启用特殊键处理
        stdscr.nodelay(True)     # 非阻塞输入
        
        self.publish_thread.start()
        
        try:
            while self.running:
                stdscr.clear()
                stdscr.addstr(0, 0, self.help_msg.format(
                    state=self.msg.state,
                    vx=self.msg.vx,
                    vy=self.msg.vy,
                    vw=self.msg.vw
                ))
                stdscr.refresh()
                
                try:
                    key = stdscr.getch()
                except:
                    key = -1
                
                if key == -1:
                    time.sleep(0.01)
                    continue
                
                if ord('0') <= key <= ord('9'):
                    self.msg.state = key - ord('0')
                    stdscr.addstr(13, 0, f"已设置 state = {self.msg.state}" + " " * 20)
                
                elif key == ord('w') or key == ord('W'):
                    self.msg.vx += self.vx_step
                    self.clamp_speed()
                    stdscr.addstr(13, 0, f"vx 增加到 {self.msg.vx:.2f} m/s" + " " * 20)
                
                elif key == ord('s') or key == ord('S'):
                    self.msg.vx -= self.vx_step
                    self.clamp_speed()
                    stdscr.addstr(13, 0, f"vx 减少到 {self.msg.vx:.2f} m/s" + " " * 20)
                
                # 新增：z键增加vy速度
                elif key == ord('z') or key == ord('Z'):
                    self.msg.vy += self.vy_step
                    self.clamp_speed()
                    stdscr.addstr(13, 0, f"vy 增加到 {self.msg.vy:.2f} m/s" + " " * 20)
                
                # 新增：c键减少vy速度
                elif key == ord('c') or key == ord('C'):
                    self.msg.vy -= self.vy_step
                    self.clamp_speed()
                    stdscr.addstr(13, 0, f"vy 减少到 {self.msg.vy:.2f} m/s" + " " * 20)
                
                elif key == ord('a') or key == ord('A'):
                    self.msg.vw += self.vw_step
                    self.clamp_speed()
                    stdscr.addstr(13, 0, f"vw 增加到 {self.msg.vw:.2f} rad/s" + " " * 20)
                
                elif key == ord('d') or key == ord('D'):
                    self.msg.vw -= self.vw_step
                    self.clamp_speed()
                    stdscr.addstr(13, 0, f"vw 减少到 {self.msg.vw:.2f} rad/s" + " " * 20)
                elif key == ord('j') or key == ord('J'):
                    self.msg.turn = 1
                elif key == ord('k') or key == ord('K'):
                    self.msg.turn = 0
                elif key == ord('l') or key == ord('L'):
                    self.msg.turn = 2

                
                elif key == ord(' '):
                    self.clear_speed()
                    stdscr.addstr(13, 0, "已清除所有速度 - 紧急停止" + " " * 20)
                
                elif key == ord('q') or key == ord('Q'):
                    stdscr.addstr(13, 0, "正在退出程序..." + " " * 20)
                    self.running = False
                
                # 刷新屏幕显示操作反馈
                stdscr.refresh()
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            self.running = False
        finally:
            # 修复：统一使用cmd_vel通道发送停止命令（原代码这里是vel_cmd，会导致停止失败）
            self.clear_speed()
            self.lcm.publish("cmd_vel", self.msg.encode())
            print("\n已发送停止命令")
            
            # 恢复终端设置
            curses.nocbreak()
            stdscr.keypad(False)
            curses.echo()
            curses.endwin()
            
            # 等待发布线程结束
            self.publish_thread.join()
            print("程序已正常退出")

def main():
    teleop = LCMTeleopKeyboard()
    # 使用curses包装器运行主函数
    curses.wrapper(teleop.run)

if __name__ == "__main__":
    main()
