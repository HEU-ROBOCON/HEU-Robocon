### 分步协作开发指南：基于分支管理与 PR 流程

---

#### **一、分支策略规划**

- **主分支 `master`**：仅存放稳定、可直接用于比赛的代码，禁止直接在此分支开发。
- **开发分支 `dev`**：用于集成阶段性功能，测试通过后再合并到 `master`。
- **个人分支**：每个成员在自己的分支（如 `feat/yourname`）开发，完成后通过 **Pull Request (PR)** 合并到 `dev` 或 `master`。

---

#### **二、首次设置分支结构**

1. **初始化主分支**：

   ```bash
   git checkout master  # 确保在 master 分支
   git push origin master  # 推送初始代码到远程
   ```
2. **创建公共开发分支 `dev`**：

   ```bash
   git checkout -b dev  # 从 master 创建 dev 分支
   git push origin dev  # 推送 dev 到远程
   ```

---

#### **三、日常开发流程（以添加新功能为例）**

##### **1. 拉取最新代码（每次开发前必做）**

```bash
git checkout master   # 切换到主分支
git pull origin master  # 拉取最新代码
```

##### **2. 创建个人分支**

```bash
git checkout -b feat/add-circle-detection  # 从 master 创建特性分支
# 分支命名建议：
# - 功能分支: feat/功能名 (如 feat/add-login)
# - 修复分支: fix/问题描述 (如 fix/color-bug)
```

##### **3. 在个人分支开发并提交**

```bash
# 修改代码后提交
git add .
git commit -m "feat: 添加红色圆形检测功能"  # 提交信息要清晰
git push origin feat/add-circle-detection  # 推送分支到远程
```

##### **4. 发起 Pull Request (PR)**

1. 进入 GitHub 仓库页面，点击 **Pull requests** → **New pull request**。
2. **选择分支**：
   - `base: master`（目标分支）
   - `compare: feat/add-circle-detection`（你的分支）
3. 填写 PR 标题和描述（说明修改内容、测试结果等）。
4. 点击 **Create pull request**，并 @ 其他人审核。

##### **5. 代码审核与合并**

- **审核人**：
  1. 查看代码变更（Files changed 标签页）。
  2. 提出评论或建议（点击行号添加评论）。
  3. 确认无误后点击 **Approve**。
- **管理员**：
  1. 确认审核通过后，点击 **Merge pull request**。
  2. 选择合并方式（推荐 **Squash and merge**，合并为单个提交）。

##### **6. 同步本地仓库**

```bash
git checkout master      # 切换回主分支
git pull origin master   # 拉取合并后的最新代码
git branch -d feat/add-circle-detection  # 删除本地已合并的分支
```

---

#### **四、处理合并冲突**

若多人修改同一文件导致冲突，按以下步骤解决：

1. 在个人分支执行：
   ```bash
   git checkout master
   git pull origin master        # 拉取最新 master
   git checkout feat/your-branch # 切回你的分支
   git merge master              # 合并 master 到当前分支
   ```
2. 手动解决冲突文件中的 `<<<<<<< HEAD` 标记。
3. 提交解决后的代码：
   ```bash
   git add .
   git commit -m "fix: 解决与 master 的冲突"
   git push origin feat/your-branch
   ```
4. PR 页面会自动更新，重新请求审核。

---

#### **五、关键注意事项**

1. **禁止直接向 `master` 提交代码**，必须通过 PR 合并。
2. **分支命名规范**：清晰表明用途（如 `feat/`, `fix/`, `docs/`）。
3. **提交信息规范**：
   - 前缀类型：`feat:`（新功能）、`fix:`（修复）、`docs:`（文档）、`chore:`（杂项）。
   - 用英文或中文简要描述修改内容。
4. **定期同步主分支**：每天开发前先 `git pull origin master` 更新本地分支。

---

#### **六、协作流程图**

```
[本地开发] → 推送分支 → [GitHub 发起 PR] → [队友审核] → [合并到 master] → [同步本地]
      ↑                                                                  ↓
      └───────────────────── 处理冲突 ←───────────────────────┘
```

---

通过以上流程，团队可以高效协作，确保 `master` 分支始终稳定可靠，同时允许成员自由开发。
