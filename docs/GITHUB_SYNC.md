# GitHub 同步说明

本项目已经配置远端仓库：`https://github.com/dunja-g/Library_robot.git`。

## 首次准备

安装并登录 GitHub CLI：

```powershell
gh auth login -h github.com
gh auth status
```

同时确认 Git 提交身份已经配置：

```powershell
git config user.name "你的 GitHub 用户名"
git config user.email "你的 GitHub 邮箱"
```

## 单次同步

在项目根目录执行：

```powershell
.\tools\sync_to_github.ps1
```

脚本会依次执行远端检查、暂存、提交和推送。没有文件变化时不会创建空提交。

## 近实时同步

```powershell
.\tools\sync_to_github.ps1 -Watch
```

默认每 8 秒检查一次。可以修改间隔：

```powershell
.\tools\sync_to_github.ps1 -Watch -IntervalSeconds 15
```

按 `Ctrl+C` 停止监听。监听只在这个 PowerShell 窗口保持运行期间有效。

## 协作安全规则

- 脚本发现远端存在本地没有的新提交时会停止，不会强制覆盖队友的工作。
- 推送失败后，本地提交仍然保留。解决登录或网络问题后重新运行即可。
- `.gitignore` 已排除虚拟环境、缓存、日志、测试照片和常见密钥文件。
- 自动同步会提交所有未忽略的工作区变化；运行前不要在项目目录中存放密码或隐私数据。
- 多人协作时，更推荐每个人使用独立分支和 Pull Request，而不是多人同时自动推送 `main`。
