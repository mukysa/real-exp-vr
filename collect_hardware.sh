
#!/bin/bash

# ==========================================
# Linux Hardware Information Collector
# Output: hardware_info.md
# ==========================================

OUTPUT="hardware_info.md"

# 辅助函数：执行命令并将结果写入 Markdown
section() {
    echo -e "\n## $1\n" >> "$OUTPUT"
    echo '```text' >> "$OUTPUT"
    shift
    if command -v "$1" >/dev/null 2>&1; then
        "$@" 2>&1 >> "$OUTPUT"
    else
        echo "Command not available: $1" >> "$OUTPUT"
    fi
    echo '```' >> "$OUTPUT"
}

# 创建 Markdown 文件
cat > "$OUTPUT" <<EOF
# Linux Hardware Information Report

- Generated: $(date '+%Y-%m-%d %H:%M:%S')
- Hostname: $(hostname)

EOF

# 1. Operating System
section "Operating System" bash -c '
cat /etc/os-release
echo
uname -a
'

# 2. CPU
section "CPU Information" lscpu

# 3. Memory
section "Memory Information" free -h

# 4. GPU
section "GPU Information" bash -c '
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi
else
    echo "NVIDIA GPU information unavailable."
fi
echo
if command -v lspci >/dev/null 2>&1; then
    lspci | grep -iE "vga|3d|display"
else
    echo "lspci unavailable."
fi
'

# 5. Disk
section "Disk Information" lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINT,MODEL

# 6. Disk Usage
section "Filesystem Usage" df -h -x tmpfs -x devtmpfs

# 7. Motherboard and BIOS
section "Motherboard Information" bash -c '
if command -v dmidecode >/dev/null 2>&1; then
    sudo -n dmidecode -t baseboard -t bios 2>/dev/null ||
    echo "Requires root permission: sudo dmidecode -t baseboard -t bios"
else
    echo "dmidecode unavailable."
fi
'

# 8. Network
section "Network Information" ip -brief address

# 9. Kernel
section "Kernel Information" uname -r

# 10. Virtualization
section "Virtualization Information" bash -c '
if command -v systemd-detect-virt >/dev/null 2>&1; then
    systemd-detect-virt
else
    echo "systemd-detect-virt unavailable."
fi
'

echo
echo "=========================================="
echo "Hardware report generated successfully!"
echo "Output: $(pwd)/$OUTPUT"
echo "=========================================="
