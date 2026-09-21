# Repository layout enforcement

The rules are executable policy in `config/repository-layout.json`.
The checker uses Python's standard library and works with the host's Python 3.8.

```bash
./setup.sh install-hooks                    # install once per clone
python3 scripts/check_layout.py --worktree  # ignored/untracked directories too
python3 scripts/check_layout.py --staged    # exact Git index, not working files
python3 scripts/check_layout.py --revision HEAD
```

Normal host-side `setup.sh` use also installs the hooks automatically. An existing
custom hooksPath or pre-commit/pre-push hook is preserved; integrate the layout
hooks with it before using setup. Shutdown and shell access remain available
even when the checkout layout is invalid.

- `pre-commit` reads the checker and policy from the index and rejects a bad
  staged tree or checkout. Deleting a bad staged file only from disk will not
  hide it. Editing the working-copy policy without staging it will not bypass it.
- `pre-push` checks every outgoing commit, including a bad intermediate commit
  subsequently removed by a clean tip.
- GitHub Actions runs the `repository-layout` check on pushes and pull requests.
  The main branch requires that check, including for administrators. Push work to
  a branch first, wait for CI, and then merge/update main with the tested commit.
  A local `--no-verify` does not satisfy the server's required status check.
- `setup.sh` rejects invalid checkout layout before generation/build/launch,
  including ignored or empty stray root directories.

Only the six visible root directories and declared root files are allowed.
Modules must use a declared group and own a module.yml; stacks must own stack.yml.
Research/results directories, tracked ws/build/log trees, output/media archives,
unapproved binaries, oversized files, symlinks and nested submodules are rejected.
Necessary runtime binary assets are exact-path exceptions with explicit size caps.
To add a real runtime asset or change ownership conventions, update the policy
and its tests in the same reviewed change.

These checks enforce structural rules, not the semantic purpose of every source
file. They also do not prohibit arbitrary local `mkdir`/file writes at the OS
level: a violation blocks the checked Git and setup operations. Repository
administrators can deliberately change the rules or branch protection.
