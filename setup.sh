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

# ------------------------------------------------------------------
# 构建模式 + module_outs 提示
# ------------------------------------------------------------------
#
# 默认 y（builtin）。理由：
#   - 本项目历史上一直是 =y（abk_control 也是 bool/builtin），现有 CI 与 Kleaf
#     配置都按 builtin 验证过，直接可用。
#   - =y 不经过 modpost，不会碰到 "device_offline undefined" 这类未导出符号问题。
#   - =y 的代价是 initcall 早于 qcom-cpufreq-hw，可能扫不到 cluster；v2.4 已加
#     扫描重试 + CPUFREQ_CREATE_POLICY 补扫 + scan sysfs 手动重扫来自愈。
#
# 若要可加载模块（.ko，可 rmmod/modprobe 重载，调试更方便）：
#     ABK_SOC_OPT_MODULE=m ./setup.sh
# 但 Kleaf 会要求把 .ko 登记进 module_outs，否则构建报：
#     ERROR: The following kernel modules are built but not copied. Add these lines
#            to the module_outs attribute of @//common:kernel_aarch64:
#                "drivers/abk_soc_opt/abk_soc_opt.ko",
# 这里不做自动改写 BUILD.bazel（容易写坏），只把需要的手动步骤打印出来。
soc_opt_enable_config() {
  local mode="${ABK_SOC_OPT_MODULE:-y}"

  abk_require_file "$DEFCONFIG"

  case "$mode" in
    m)
      abk_module_config CONFIG_ABK_SOC_OPT "$DEFCONFIG"
      ;;
    y|"")
      mode="y"
      abk_enable_config CONFIG_ABK_SOC_OPT "$DEFCONFIG"
      ;;
    *)
      abk_die "ABK_SOC_OPT_MODULE 只能是 y 或 m，收到: $mode"
      ;;
  esac

  abk_log "CONFIG_ABK_SOC_OPT=$mode set in $DEFCONFIG"
}

soc_opt_module_outs_hint() {
  local common_dir build_file

  common_dir="$(soc_opt_common_dir)"
  build_file="$common_dir/BUILD.bazel"

  abk_log "---------------------------------------------------------------"
  abk_log "选择了 =m（可加载模块），Kleaf 需要登记 module_outs，否则会报"
  abk_log "  'The following kernel modules are built but not copied'"
  abk_log ""
  abk_log "请在 $build_file 的 kernel_aarch64 规则里加入："
  abk_log "    module_outs = ["
  abk_log "        \"drivers/abk_soc_opt/abk_soc_opt.ko\","
  abk_log "    ],"
  abk_log ""
  abk_log "或安装 buildozer 后执行："
  abk_log "    buildozer 'add module_outs drivers/abk_soc_opt/abk_soc_opt.ko' \\"
  abk_log "        @//common:kernel_aarch64"
  abk_log ""
  abk_log "不想动 Bazel 配置的话，去掉 ABK_SOC_OPT_MODULE=m 用默认的 =y 即可。"
  abk_log "---------------------------------------------------------------"

  if [ -f "$build_file" ] && grep -qF 'drivers/abk_soc_opt/abk_soc_opt.ko' "$build_file"; then
    abk_log "检测到 BUILD.bazel 里已包含该 .ko，无需手动修改。"
  fi
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
    abk_log "before_build: enabling CONFIG_ABK_SOC_OPT (mode=${ABK_SOC_OPT_MODULE:-y})"
    soc_opt_install_kernel_files
    soc_opt_enable_config
    if [ "${ABK_SOC_OPT_MODULE:-}" = "m" ]; then
      soc_opt_module_outs_hint
    fi
    ;;

  *)
    abk_die "unsupported CUSTOM_EXTERNAL_MODULE_STAGE: $CUSTOM_EXTERNAL_MODULE_STAGE"
    ;;
esac

abk_log "done"
