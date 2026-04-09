#!/bin/bash
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BACKUP_DIR="$ROOT_DIR/runtime/backups/anc-kernel-tuning-$(date +%Y%m%d-%H%M%S)"
BOOT_CMDLINE="/boot/firmware/cmdline.txt"
SYSCTL_DST="/etc/sysctl.d/99-anc-rt.conf"
IRQ_SERVICE_DST="/etc/systemd/system/anc-irq-affinity.service"
ANC_SERVICE_DST="/etc/systemd/system/anc-rt.service"
CPU_SERVICE_DST="/etc/systemd/system/cpu-performance.service"
SYSTEMD_CONF_DST="/etc/systemd/system.conf.d/20-anc-housekeeping.conf"
IRQ_SCRIPT_DST="/usr/local/sbin/anc-irq-affinity.sh"

mkdir -p "$BACKUP_DIR" "$(dirname "$SYSTEMD_CONF_DST")"

cp -a "$BOOT_CMDLINE" "$BACKUP_DIR/" 2>/dev/null || true
cp -a "$SYSCTL_DST" "$BACKUP_DIR/" 2>/dev/null || true
cp -a "$IRQ_SERVICE_DST" "$BACKUP_DIR/" 2>/dev/null || true
cp -a "$ANC_SERVICE_DST" "$BACKUP_DIR/" 2>/dev/null || true
cp -a "$CPU_SERVICE_DST" "$BACKUP_DIR/" 2>/dev/null || true
cp -a "$SYSTEMD_CONF_DST" "$BACKUP_DIR/" 2>/dev/null || true
cp -a "$IRQ_SCRIPT_DST" "$BACKUP_DIR/" 2>/dev/null || true

install -m 0755 "$ROOT_DIR/scripts/anc-irq-affinity.sh" "$IRQ_SCRIPT_DST"
install -m 0644 "$ROOT_DIR/systemd/anc-irq-affinity.service" "$IRQ_SERVICE_DST"
install -m 0644 "$ROOT_DIR/systemd/anc-rt.service" "$ANC_SERVICE_DST"
install -m 0644 "$ROOT_DIR/systemd/cpu-performance.service" "$CPU_SERVICE_DST"
install -m 0644 "$ROOT_DIR/config/anc-rt.conf" "$SYSCTL_DST"
install -m 0644 "$ROOT_DIR/config/anc-system.conf" "$SYSTEMD_CONF_DST"

cmdline="$(cat "$BOOT_CMDLINE")"
for token in \
    "threadirqs" \
    "nohz_full=3" \
    "rcu_nocbs=3" \
    "irqaffinity=0-2" \
    "isolcpus=domain,managed_irq,3"
do
    case " $cmdline " in
        *" $token "*) ;;
        *) cmdline="$cmdline $token" ;;
    esac
done
printf '%s\n' "$cmdline" > "$BOOT_CMDLINE"

sysctl --load "$SYSCTL_DST"
systemctl daemon-reload
systemctl enable anc-irq-affinity.service cpu-performance.service anc-rt.service
systemctl restart anc-irq-affinity.service
systemctl restart cpu-performance.service

echo "Applied ANC kernel tuning."
echo "Reboot required for cmdline isolation parameters to take effect."
echo "Backups saved to $BACKUP_DIR"
