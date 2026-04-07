# Codex Workflow

- Before making any software changes, explicitly state the current git branch to the user.
- Keep this branch announcement in the commentary/update message that comes before the edit.
- On `Development2`, do not automatically commit changes; leave them uncommitted for the user to review and commit.
- For `Development2` to `main` branch syncs, use `.\merge-development2-into-main.ps1` so the merge always includes the line-ending renormalization step before commit.
