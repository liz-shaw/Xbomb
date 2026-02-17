###  基础初始化操作流程

#### 1. 初始化本地仓库

Bash

```
cd ~/your_ws_path/
git init
```

- **逻辑**：在当前文件夹创建一个隐藏的 `.git` 目录，开启版本控制。

#### 2. 配置忽略文件 (必做)

ROS 2 编译产生的 `build/`, `install/`, `log/` 包含成千上万个中间文件，**严禁**上传。

Bash

```
cat <<EOF > .gitignore
build/
install/
log/
.vscode/
__pycache__/
*.pyc
EOF
```

#### 3. 设置身份信息 (解决 "Author identity unknown")

每次提交都会记录贡献者，第一次使用 Git 必须配置。

Bash

```
git config --global user.email "your_email@example.com"
git config --global user.name "username"
```

#### 4. 提交代码到本地

Bash

```
git add .
git commit -m "feat: 首次提交 ROS 2 基础节点代码"
```

#### 5. 解决身份验证 (SSH Key 握手)

如果 `git push` 提示 `Permission denied (publickey)`：

1. **生成钥匙对**：`ssh-keygen -t ed25519 -C "email"` (一路回车)。
2. **获取公钥**：`cat ~/.ssh/id_ed25519.pub`。
3. **配置**：复制内容粘贴到 GitHub 网页的 `Settings -> SSH and GPG keys` 中。

#### 6. 关联远程仓库并推送

Bash

```
# 关联远程地址
git remote add origin git@github.com:username/reponame.git

# 将本地分支 master 重命名为 main (符合 GitHub 默认习惯)
git branch -M main

# 解决冲突 (如果云端有 README 而本地没有)
git pull origin main --rebase

# 正式推送
git push -u origin main
```

------

###  Git 命令行结构逻辑与标识符

Git 的命令结构通常遵循：`git <动词> <选项/标志> <参数>`。

| **标识符/参数** | **含义说明**         | **形象理解**                                           |
| --------------- | -------------------- | ------------------------------------------------------ |
| **`init`**      | 初始化               | 圈出一块地准备盖房子。                                 |
| **`add .`**     | 添加当前目录所有修改 | 把要搬家搬走的东西装箱，`.` 代表全部。                 |
| **`commit`**    | 提交                 | 封箱并贴上标签。                                       |
| **`-m "msg"`**  | Message 标志         | 标签上写的文字说明（必须写）。                         |
| **`remote`**    | 远程                 | 建立你的电脑和 GitHub 服务器的联系。                   |
| **`origin`**    | 远程主机的默认代号   | 相当于给远程仓库地址起个外号叫 origin。                |
| **`push`**      | 推送                 | 把箱子发货到云端仓库。                                 |
| **`-u`**        | 建立追踪 (Upstream)  | 第一次用以后，下次直接输入 `git push` 就行。           |
| **`main`**      | 分支名               | 你的代码“主干道”。                                     |
| **`--rebase`**  | 变基/重新整理历史    | 把云端的文件先插到你代码的最前面，保证历史成一条直线。 |

------

### 开发中高频的 Git 命令

- **查看当前状态**：

  `git status` (随时查看哪些文件改了没提交)

- **查看提交历史**：

  `git log --oneline` (查看简洁的提交记录)

- **撤销上一次 add**：

  `git reset .` (发现加错文件了，可以退回)

- **放弃所有本地修改**（救命药）：

  `git checkout -- .` (代码改乱了想回到上次 commit 的状态)

- **创建新分支**：

  `git checkout -b dev` (在开发新功能如“雷达驱动”时，先不在 main 分支乱动)

- **合并分支**：

  `git merge dev` (新功能测试好了，合并回主分支)

- 删除 Git 索引中的所有缓存-

  - 仅删索引，不删本地物理文件
  -  `git rm -r --cached .`

------

### 常见调试排错

1. **分支名不一致**：本地叫 `master`，云端叫 `main`。解决方法：`git branch -M main`。
2. **网络或公钥问题**：提示 `fatal: Could not read from remote...`。解决方法：重新检查 SSH key 是否添加成功，输入 `ssh -T git@github.com` 测试连接。
3. **非快进式错误**：提示 `[rejected]`。原因：云端有你本地没有的文件。解决方法：`git pull --rebase`。

