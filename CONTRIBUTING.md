# 贡献指南

本 fork 的适配分支是 `honkai-rendering/ue5.8`。PR 请以该分支为目标；通用上游问题先确认是否能在未修改的 VRM4U 复现，并遵循上游贡献方式。

问题报告请注明 UE、插件提交、Windows/GPU、最小复现步骤、预期和实际结果及脱敏日志。不要上传没有分发权的模型、贴图或人物录像；使用合法来源链接和最小自制样例。

保留 VRM4U、Assimp、Unity MMD Tools 和 Bullet 的署名与许可。PMX 修复应依据格式、骨骼及权重，禁止按角色名或素材路径硬编码。运行宿主 `VRM4U.PMX` 测试；涉及物理时还需 native 测试及多个模型的实际网格回归。PR 中列出已执行和未执行的验证。

请遵循[社区行为准则](CODE_OF_CONDUCT.md)。漏洞按[安全报告说明](SECURITY.md)处理。
