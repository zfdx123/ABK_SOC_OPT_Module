#!/usr/bin/env bash
set -euo pipefail

MODULE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$MODULE_DIR/module.conf" ]; then
  # shellcheck disable=SC1091
  source "$MODULE_DIR/module.conf"
fi

# shellcheck disable=SC1091
source "$MODULE_DIR/scripts/libabk.sh"

abk_require_env KERNEL_ROOT DEFCONFIG CUSTOM_EXTERNAL_MODULE_STAGE

abk_log "module: ${ABK_MODULE_NAME:-ABK SoC Opt}"
abk_log "version: ${ABK_MODULE_VERSION:-unknown}"
abk_log "stage: $CUSTOM_EXTERNAL_MODULE_STAGE"
abk_log "kernel root: $KERNEL_ROOT"

KERNEL_DRIVER_DIR="$KERNEL_ROOT/drivers/soc/abk_soc_opt"
KERNEL_SOC_KCONFIG="$KERNEL_ROOT/drivers/soc/Kconfig"
KERNEL_SOC_MAKEFILE="$KERNEL_ROOT/drivers/soc/Makefile"

case "$CUSTOM_EXTERNAL_MODULE_STAGE" in
  after_patch)
    # ---------------------------------------------------------------
    # 1. 复制源码到内核树 drivers/soc/abk_soc_opt/
    # ---------------------------------------------------------------
    abk_log "copying source files to $KERNEL_DRIVER_DIR ..."
    mkdir -p "$KERNEL_DRIVER_DIR"
    cp -a "$MODULE_DIR/files/abk_soc_opt.c" "$KERNEL_DRIVER_DIR/"
    cp -a "$MODULE_DIR/files/Kconfig"       "$KERNEL_DRIVER_DIR/"
    cp -a "$MODULE_DIR/files/Makefile"      "$KERNEL_DRIVER_DIR/"
    abk_log "copied: abk_soc_opt.c Kconfig Makefile"

    # ---------------------------------------------------------------
    # 2. 在 drivers/soc/Kconfig 中 source 我们的 Kconfig
    # ---------------------------------------------------------------
    abk_require_file "$KERNEL_SOC_KCONFIG"
    abk_append_line_once "$KERNEL_SOC_KCONFIG" \
      'source "drivers/soc/abk_soc_opt/Kconfig"'

    # ---------------------------------------------------------------
    # 3. 在 drivers/soc/Makefile 中注册编译目标
    # ---------------------------------------------------------------
    abk_require_file "$KERNEL_SOC_MAKEFILE"
    abk_append_line_once "$KERNEL_SOC_MAKEFILE" \
      'obj-$(CONFIG_ABK_SOC_OPT) += abk_soc_opt/'

    abk_log "after_patch done — source injected at drivers/soc/abk_soc_opt/"
    ;;

  before_build)
    # ---------------------------------------------------------------
    # 启用 CONFIG_ABK_SOC_OPT=m
    # ---------------------------------------------------------------
    abk_require_file "$DEFCONFIG"
    # abk_module_config CONFIG_ABK_SOC_OPT "$DEFCONFIG"
    abk_enable_config CONFIG_ABK_SOC_OPT "$DEFCONFIG"
    abk_log "before_build done — CONFIG_ABK_SOC_OPT=m enabled"
    ;;

  *)
    abk_die "unsupported CUSTOM_EXTERNAL_MODULE_STAGE: $CUSTOM_EXTERNAL_MODULE_STAGE"
    ;;
esac

abk_log "done"
