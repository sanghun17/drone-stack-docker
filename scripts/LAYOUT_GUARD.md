# Repository layout enforcement

The rules are executable policy in `config/repository-layout.json`.
The checker uses Python's standard library and works with the host's Python 3.8.

```bash
./setup.sh install-hooks                    # install Git hooks AND root creation lock
./setup.sh root-status                      # inspect actual permission state
python3 scripts/check_layout.py --worktree  # ignored/untracked directories too
python3 scripts/check_layout.py --staged    # exact Git index, not working files
python3 scripts/check_layout.py --revision HEAD
```

Normal host-side `setup.sh` use also installs the hooks and root lock automatically. An existing
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
Misplaced outputs, tracked module implementations and data payloads, tracked ws/build/log trees, output/media archives,
unapproved binaries, oversized files, symlinks and nested submodules are rejected.
Training modules are outside this runtime repository. `forbidden_path_prefixes`
rejects the retired training stacks, controller/pose wrappers and device-backup
location, including ignored directories. These nested paths are rejected by the
layout checks and Git hooks; immediate filesystem blocking applies only at the root.
Necessary runtime binary assets are exact-path exceptions with explicit size caps.
To add a real runtime asset or change ownership conventions, update the policy
and its tests in the same reviewed change.

## Immediate creation protection

`scripts/root_guard.py lock` removes all write bits from the checkout root inode
(normally 0775 becomes 0555). The kernel rejects top-level `mkdir`, new files,
symlinks, renames and deletions regardless of which agent/tool performs them.
Only that inode changes: normal edits and new directories inside `modules`,
`scripts`, `stacks`, `config`, `data`, `ws`, `flight_logs`, `.build` and `.git` still work.
The allowed workspace/log/build roots are created before locking a fresh clone.

The lock survives agent restarts and reboot. It is local filesystem state, not a
Git-tracked permission, so activate it on each new clone. Original permissions
and inode identity are recorded in the checkout's Git metadata; repeated locking
does not overwrite the original mode. `./setup.sh unlock-root` deliberately
disables the lock and restores that mode; normal setup usage enables it again.

Existing root file contents can be edited in place, but editors that save by
replacing/renaming the file, and Git updates to root files, require maintenance:

```bash
python3 scripts/root_guard.py maintain -- git pull --ff-only
# Or run the explicitly authorized root-file editor command after `maintain --`.
```

This opens a temporary write window, checks the resulting layout, and relocks
even when the command fails. Do not run concurrent agents in that window. A
forced SIGKILL or power loss during maintenance cannot run cleanup; use
`./setup.sh lock-root` to restore protection. A violating maintenance command is
reported without deleting its output. Shutdown and shell access stay available.

This is an accidental-creation barrier, not a hostile-agent sandbox. The owner
can deliberately chmod/unlock the directory; root/Docker privileges can bypass
permissions, and a writable parent directory allows replacement of the entire
checkout. Agents must not do these things to bypass a rejection. Strict isolation
against a hostile agent requires a separate restricted identity/sandbox.
Nested directories remain writable and are checked by the Git/layout guards;
the permission lock does not classify the semantic purpose of every source file.

Project analysis source and metadata are allowed under data/analysis and
data/manifests. The data/assets, data/results and data/archive roots are local
payload storage; force-adding their files is rejected by the index check.
Module snapshots are owned by their locked remote packages, not this Git index.
