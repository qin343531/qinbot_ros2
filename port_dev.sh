#!/bin/bash
# port_dev.sh - 绑定物理USB口生成固定设备号（适配泰山派，仅2个参数）
# 使用方式：sudo ./port_dev.sh <临时设备路径> <目标软链接>
# 示例：sudo ./port_dev.sh /dev/ttyUSB0 /dev/ydlidar

# ====================== 检查参数 ======================
if [ $# -ne 2 ]; then
    echo -e "\033[31m❌ 参数错误！正确用法：\033[0m"
    echo "sudo $0 <临时设备路径> <目标软链接>"
    echo "示例：sudo $0 /dev/ttyUSB0 /dev/ydlidar"
    exit 1
fi

# 提取输入参数
TEMP_DEV="$1"       # 临时设备路径（如/dev/ttyUSB0）
TARGET_LINK="$2"    # 目标软链接（如/dev/ydlidar）
LINK_NAME=$(basename "$TARGET_LINK")  # 提取软链接名称（如ydlidar）

# 检查临时设备是否存在
if [ ! -c "$TEMP_DEV" ]; then
    echo -e "\033[31m❌ 临时设备 $TEMP_DEV 不存在！请先接入设备并确认路径。\033[0m"
    exit 1
fi

# ====================== 提取核心标识 ======================
echo -e "\033[34m🔍 正在提取设备 $TEMP_DEV 的核心标识...\033[0m"

# 1. 提取物理USB端口号（关键：区分不同USB口）
PHYSICAL_PORT=$(udevadm info --attribute-walk --name="$TEMP_DEV" | grep -E 'KERNELS=="[0-9]+-[0-9.]+"' | head -n1 | awk -F'"' '{print $2}')
if [ -z "$PHYSICAL_PORT" ]; then
    echo -e "\033[31m❌ 无法提取物理USB端口号！\033[0m"
    exit 1
fi
echo -e "\033[32m✅ 提取到物理USB端口号：$PHYSICAL_PORT\033[0m"

# 2. 提取VID/PID（适配泰山派：改用lsusb+设备路径关联，更稳定）
# 先获取设备的bus和device号
DEV_BUS=$(udevadm info --query=property --name="$TEMP_DEV" | grep 'BUSNUM=' | cut -d'=' -f2)
DEV_DEV=$(udevadm info --query=property --name="$TEMP_DEV" | grep 'DEVNUM=' | cut -d'=' -f2)

if [ -n "$DEV_BUS" ] && [ -n "$DEV_DEV" ]; then
    # 通过lsusb提取VID/PID
    USB_INFO=$(lsusb -s "$DEV_BUS:$DEV_DEV" | awk '{print $6}')
    VID=$(echo "$USB_INFO" | cut -d':' -f1)
    PID=$(echo "$USB_INFO" | cut -d':' -f2)
else
    # 兜底方案：从udevadm输出中模糊提取（兼容更多格式）
    VID=$(udevadm info --attribute-walk --name="$TEMP_DEV" | grep -i 'idvendor' | head -n1 | awk -F'"' '{print $2}')
    PID=$(udevadm info --attribute-walk --name="$TEMP_DEV" | grep -i 'idproduct' | head -n1 | awk -F'"' '{print $2}')
fi

# 检查VID/PID是否提取成功
if [ -z "$VID" ] || [ -z "$PID" ]; then
    echo -e "\033[33m⚠️  无法自动提取VID/PID，将跳过VID/PID匹配（仅按物理端口绑定）\033[0m"
    VID=""
    PID=""
else
    echo -e "\033[32m✅ 提取到VID/PID：$VID/$PID\033[0m"
fi

# ====================== 生成udev规则 ======================
# 规则文件（优先级99，避免被覆盖）
UDEV_RULE_FILE="/etc/udev/rules.d/99-usb-physical-port.rules"
# 清空旧规则（避免冲突，只保留当前绑定）
echo "# 自动生成的物理USB端口绑定规则（$(date +%Y-%m-%d)）" > "$UDEV_RULE_FILE"

# 构建规则（无VID/PID时仅按物理端口匹配）
if [ -n "$VID" ] && [ -n "$PID" ]; then
    RULE_ADD="SUBSYSTEM==\"tty\", KERNELS==\"$PHYSICAL_PORT\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", ACTION==\"add\", SYMLINK+=\"$LINK_NAME\", MODE=\"0666\""
    RULE_REMOVE="SUBSYSTEM==\"tty\", KERNELS==\"$PHYSICAL_PORT\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", ACTION==\"remove\", RUN+=\"/bin/rm -f $TARGET_LINK\""
else
    RULE_ADD="SUBSYSTEM==\"tty\", KERNELS==\"$PHYSICAL_PORT\", ACTION==\"add\", SYMLINK+=\"$LINK_NAME\", MODE=\"0666\""
    RULE_REMOVE="SUBSYSTEM==\"tty\", KERNELS==\"$PHYSICAL_PORT\", ACTION==\"remove\", RUN+=\"/bin/rm -f $TARGET_LINK\""
fi

echo "$RULE_ADD" >> "$UDEV_RULE_FILE"
echo "$RULE_REMOVE" >> "$UDEV_RULE_FILE"

# ====================== 生效规则并验证 ======================
echo -e "\033[32m📝 已生成udev规则文件：$UDEV_RULE_FILE\033[0m"
if [ -n "$VID" ] && [ -n "$PID" ]; then
    echo -e "\033[32m📌 绑定规则：仅物理端口 $PHYSICAL_PORT + VID/PID $VID/$PID 的设备会创建 $TARGET_LINK\033[0m"
else
    echo -e "\033[32m📌 绑定规则：仅物理端口 $PHYSICAL_PORT 的串口设备会创建 $TARGET_LINK\033[0m"
fi

# 重载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 立即创建软链接（无需重新插拔）
if [ -L "$TARGET_LINK" ]; then
    sudo rm -f "$TARGET_LINK"
fi
sudo ln -s "$TEMP_DEV" "$TARGET_LINK"
sudo chmod 666 "$TARGET_LINK"

# 最终验证
echo -e "\n\033[32m✅ 配置完成！验证结果：\033[0m"
if [ -L "$TARGET_LINK" ]; then
    REAL_DEV=$(readlink "$TARGET_LINK")
    echo "   软链接已创建：$TARGET_LINK -> $REAL_DEV"
    echo "   仅当设备插入物理端口 $PHYSICAL_PORT 时，才会自动生成该软链接"
else
    echo -e "\033[33m⚠️  软链接创建失败，请重新插拔设备后重试！\033[0m"
fi