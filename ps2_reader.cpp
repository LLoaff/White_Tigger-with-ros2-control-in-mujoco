#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <unistd.h>

static const char* keyName(unsigned short code) {
    // 这是该 0810:0001 适配器的常见 Linux 映射。
    switch (code) {
        case BTN_TRIGGER: return "Triangle";
        case BTN_THUMB:   return "Circle";
        case BTN_THUMB2:  return "Cross";
        case BTN_TOP:     return "Square";
        case BTN_TOP2:    return "L2";
        case BTN_PINKIE:  return "R2";
        case BTN_BASE:    return "L1";
        case BTN_BASE2:   return "R1";
        case BTN_BASE3:   return "Select";
        case BTN_BASE4:   return "Start";
        case BTN_BASE5:   return "L3";
        case BTN_BASE6:   return "R3";
        default:          return "Unknown";
    }
}

static const char* axisName(unsigned short code) {
    switch (code) {
        case ABS_X:     return "Left X";
        case ABS_Y:     return "Left Y";
        case ABS_Z:     return "Right X";
        case ABS_RZ:    return "Right Y";
        case ABS_HAT0X: return "DPad X";
        case ABS_HAT0Y: return "DPad Y";
        default:        return "Unknown axis";
    }
}

static float normalizeAxis(int value, const input_absinfo& info) {
    const float center = (info.minimum + info.maximum) * 0.5f;

    if (value >= center) {
        return (value - center) / (info.maximum - center);
    }
    return (value - center) / (center - info.minimum);
}

static float deadZone(float value, float zone = 0.08f) {
    if (std::fabs(value) < zone) {
        return 0.0f;
    }

    // 去掉死区后重新拉伸到 -1.0 ~ 1.0
    return (value > 0)
        ? (value - zone) / (1.0f - zone)
        : (value + zone) / (1.0f - zone);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "用法: %s /dev/input/eventN\n", argv[0]);
        return 1;
    }

    const char* device = argv[1];
    int fd = open(device, O_RDONLY | O_CLOEXEC);

    if (fd < 0) {
        std::perror("无法打开输入设备");
        return 1;
    }

    char name[256] = {};
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0) {
        std::printf("设备名称: %s\n", name);
    }

    input_id id {};
    if (ioctl(fd, EVIOCGID, &id) == 0) {
        std::printf("USB ID: %04x:%04x\n", id.vendor, id.product);
    }

    // 读取各轴的原始范围，不能假定所有手柄都是 -32767 ~ 32767。
    input_absinfo axisInfo[ABS_CNT] {};
    for (int axis = 0; axis < ABS_CNT; ++axis) {
        ioctl(fd, EVIOCGABS(axis), &axisInfo[axis]);
    }

    float leftX = 0.0f;
    float leftY = 0.0f;
    float rightX = 0.0f;
    float rightY = 0.0f;

    std::puts("开始读取手柄。按 Ctrl+C 退出。");

    while (true) {
        pollfd pfd {fd, POLLIN, 0};

        int ready = poll(&pfd, 1, -1);
        if (ready < 0) {
            if (errno == EINTR) continue;
            std::perror("poll");
            break;
        }

        input_event ev {};
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n < 0) {
            if (errno == EINTR) continue;
            std::perror("read");
            break;
        }
        if (n != sizeof(ev)) {
            continue;
        }

        if (ev.type == EV_KEY) {
            // value: 1=按下，0=松开，2=长按自动重复
            std::printf("按键 %-9s code=%u value=%d\n",
                        keyName(ev.code), ev.code, ev.value);
        }

        if (ev.type == EV_ABS) {
            if (ev.code == ABS_HAT0X || ev.code == ABS_HAT0Y) {
                // 十字键：-1、0、1
                std::printf("十字键 %-7s value=%d\n",
                            axisName(ev.code), ev.value);
                continue;
            }

            float value = deadZone(normalizeAxis(ev.value, axisInfo[ev.code]));

            switch (ev.code) {
                case ABS_X:  leftX = value; break;
                case ABS_Y:  leftY = value; break;
                case ABS_Z:  rightX = value; break;
                case ABS_RZ: rightY = value; break;
                default: break;
            }

            std::printf(
                "摇杆 %-8s raw=%6d | L=(%+.2f, %+.2f) R=(%+.2f, %+.2f)\n",
                axisName(ev.code), ev.value,
                leftX, leftY, rightX, rightY
            );

            // 机器人控制示例：
            float forward = -leftY;  // 有些手柄上推是负数，所以取反
            float turn = rightX;

            float linearSpeed = forward * 1.0f;  // 最高 1.0 m/s，按你的底盘改
            float angularSpeed = turn * 1.5f;    // 最高 1.5 rad/s，按你的底盘改

            // 在这里替换为你的底盘命令：
            // robot.setVelocity(linearSpeed, angularSpeed);
            (void)linearSpeed;
            (void)angularSpeed;
        }

        if (ev.type == EV_SYN && ev.code == SYN_DROPPED) {
            // 事件缓存溢出：机器人项目中必须立即停车，避免保留旧速度。
            std::fprintf(stderr, "输入事件溢出，建议立即停车并重新读取状态\n");

            leftX = leftY = rightX = rightY = 0.0f;
            // robot.stop();
        }
    }

    close(fd);
    return 0;
}