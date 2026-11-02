# 贡献约定

## 提交

一次提交只做一件可独立验证的事。提交前必须过门禁：

```
python tests/test_model.py
python tests/test_ik.py
python tests/test_traj.py
python tools/validate_urdf.py --deep
python src/robot/demos/run_all.py
```

任一失败不提交。

## 目录

根下只放元文件（README、CONTRIBUTING、LICENSE）、工具配置（`.gitignore`、`.gitattributes`）
和五个角色目录：`src/` `tests/` `tools/` `data/` `external/`。
新增文件按角色入桶，不放根目录。临时脚本用完即删，不提交。

## 路径

脚本里不写死绝对路径。仓库根用向上找 `.git` 的方式解析，数据路径统一从 `r4ctl/config.py`
的 `ROOT` 派生。

## 文档

只维护 `README.md` 一份。同一事实只写一处，避免漂移。
