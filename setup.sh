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

# ------------------------------------------------------------------
# 工具函数
# ------------------------------------------------------------------

soc_opt_common_dir() {
  abk_common_dir
}

soc_opt_copy_tree() {
  local src="$1" dst="$2"
  abk_require_dir "$src"
  mkdir -p "$dst"
  cp -a "$src"/. "$dst"/
  abk_log "synced $src -> $dst"
}

# ------------------------------------------------------------------
# 内核树注入
# ------------------------------------------------------------------

soc_opt_install_kernel_files() {
  local common_dir drivers_dir

  common_dir="$(soc_opt_common_dir)"
  drivers_dir="$common_dir/drivers"

  abk_require_dir "$drivers_dir"
  abk_require_file "$drivers_dir/Kconfig"
  abk_require_file "$drivers_dir/Makefile"

  # 复制模块源码
  soc_opt_copy_tree \
    "$MODULE_DIR/files" \
    "$drivers_dir/abk_soc_opt"

  # 注册 Kconfig
  abk_append_line_once "$drivers_dir/Kconfig" \
    'source "drivers/abk_soc_opt/Kconfig"'

  # 注册 Makefile
  abk_append_line_once "$drivers_dir/Makefile" \
    'obj-$(CONFIG_ABK_SOC_OPT) += abk_soc_opt/'

  abk_log "kernel files installed at drivers/abk_soc_opt/"
}

soc_opt_enable_config() {
  abk_require_file "$DEFCONFIG"
  abk_enable_config CONFIG_ABK_SOC_OPT "$DEFCONFIG"
  abk_log "CONFIG_ABK_SOC_OPT=y enabled in $DEFCONFIG"
}

# ------------------------------------------------------------------
# 入口
# ------------------------------------------------------------------

case "$CUSTOM_EXTERNAL_MODULE_STAGE" in
  after_patch)
    abk_log "after_patch: installing ABK SoC Opt kernel driver"
    soc_opt_install_kernel_files
    ;;

  before_build)
    abk_log "before_build: enabling CONFIG_ABK_SOC_OPT=m"
    soc_opt_enable_config
    ;;

  *)
    abk_die "unsupported CUSTOM_EXTERNAL_MODULE_STAGE: $CUSTOM_EXTERNAL_MODULE_STAGE"
    ;;
esac

abk_log "done"
