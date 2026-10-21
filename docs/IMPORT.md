# 整理范围与来源

整理日期：2026-10-21。

- `projects/EYETLE` 来自 `/home/whill/projects/EYETLE` 的当前工作副本，上游 https://github.com/Yongying-Zhu/EYETLE 。
- `projects/Ble2WChair` 来自 `/home/whill/projects/Ble2WChair` 的当前工作副本，上游 https://github.com/Yongying-Zhu/Ble2WChair 。MIT 许可保留在组件目录。
- `wheelchair_shared_ws` 来自同名本地工作空间，含本地最新控制、适配器、仿真及测试代码。
- `projects/*.md` 保留原始设计、接口与实施记录，避免打断现有相对链接。

使用工作副本快照，包含未提交的最新源码；不嵌套原 `.git`，不修改原目录。导入文件映射见 [import-manifest.json](import-manifest.json)。新增顶层文档及入口不列入原始文件映射。

排除构建安装目录、虚拟环境、日志、测试产物、个人标定、相机录像、模型二进制、第三方下载目录与已安装运行库。保留模型构建脚本、来源哈希、运行库版本清单及所用场景许可；没有将测试历史结果作为本次验收。

EYETLE、PuReST、RITnet、场景资源等按各自许可使用；不将整个仓库笼统重授权为 MIT。各 package.xml 的许可证声明和资源内原始许可继续保留。

## 远程已有版本

基于远程 main 提交 `6b975fa06652c1ebf50a54913bcafc9a50b363be` 整理。原远程全部文件原样迁至 `platforms/jazzy/`，包括已经被远程跟踪的 ONNX / H5 等模型和恢复记录；本次不删除这些已有资产。顶层新增本机完整 Humble 源码。Jazzy 脚本按自身相对路径定位，运行时先进入该子目录。原 README 内的 `~/whill` 为历史机器路径，应替换成此子目录实际路径。

两套平台实现保留独立副本以避免把已恢复的 Jazzy 兼容修改混进 Humble 控制链路；未来可以在跨平台验证后再提取公共包。本次没有在 Ubuntu 24.04 上重验 Jazzy。
