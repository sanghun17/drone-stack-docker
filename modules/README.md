# Remote module packages

This directory is a local deployment cache. Only this document and SCHEMA.md are
tracked by the stack repository. Module installation, configuration, runtime code
and artifact declarations live in their owning repositories.

Run `./setup.sh sync <stack>` to materialize the exact packages from
`config/modules.lock.json`. To change a module, edit its owning Git repository,
commit/push there, update the lock and sync. Local snapshot edits are preserved
and rejected rather than silently overwritten. See the root README for ownership
and update commands.
