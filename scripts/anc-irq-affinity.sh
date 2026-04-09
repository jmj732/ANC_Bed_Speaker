#!/bin/bash
set -eu

HOUSEKEEPING_CPUS="${HOUSEKEEPING_CPUS:-0-2}"

set_affinity() {
    local path="$1"
    if [ -w "$path" ]; then
        printf '%s' "$HOUSEKEEPING_CPUS" > "$path" || true
    fi
}

for path in /proc/irq/*/smp_affinity_list; do
    [ -e "$path" ] || continue
    current="$(cat "$path" 2>/dev/null || true)"
    case "$current" in
        0-3|0,1,2,3)
            set_affinity "$path"
            ;;
    esac
done

for irq_dir in /proc/irq/[0-9]*; do
    [ -d "$irq_dir" ] || continue
    actions="$irq_dir/actions"
    [ -r "$actions" ] || continue
    if grep -Eq 'dw_axi_dmac_platform|xhci-hcd|mmc|sdhci|pcie' "$actions"; then
        set_affinity "$irq_dir/smp_affinity_list"
    fi
done
