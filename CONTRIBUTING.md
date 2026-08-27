# Contributing

Everyone works on their own branch and merges into `main` via pull request. Nobody pushes to
`main` directly.

## 1. Branch naming

```
<your-name>/<package>-<short-description>
```

Examples: `sohan/rover_slam-rtabmap-bringup`, `ram/rover_drivers-zed2i-node`.

## 2. Stay current with `main`

**Do not wait to be told that `main` moved.** Fetch at the start of every work session, and again
before you open a PR:

```bash
git fetch origin
git rebase origin/main
```

If you have already pushed the branch and then rebase, push with
`git push --force-with-lease` (never a plain `--force`).

`main` moves whenever someone's PR merges, so it will be ahead of your branch regularly. Rebasing
early keeps conflicts to a few lines instead of a few hundred.

## 3. Keep your PR inside your own package

Stay within your own `src/<package>/` wherever you can — that is what makes parallel work by ten
people possible without constant conflicts.

Three areas are shared, and touching them can silently break other people's branches:

- `src/rover_interfaces/` — message, service, and action definitions
- `docker/` — everyone's build environment
- root `README.md` / `CONTRIBUTING.md`

If your PR changes any of these, say so explicitly in the PR description, and announce it in the
team channel once it merges, so everyone knows to rebase.

## 4. Pull requests

- Open against `main`.
- At least one approval before merge.
- Describe what changed, what you tested it against (real hardware, Gazebo, recorded bag), and
  any shared-area changes per section 3.
- Rebase onto the latest `origin/main` before asking for the merge.

## 5. Adding build files to a placeholder package

Every `src/<package>/` currently holds only a README. The owner's first PR adds:

- `package.xml`
- `CMakeLists.txt` (for `ament_cmake` / C++) or `setup.py` + `setup.cfg` (for `ament_python`)

and keeps the README, updating the Status line.

---

## Repo admin setup (for the maintainer)

Sections 2 and 4 are conventions; make them enforced so a missed announcement cannot cause a bad
merge. On GitHub: **Settings → Rules → Rulesets → New branch ruleset**, targeting `main`:

- ✅ Require a pull request before merging
- ✅ Require approvals: **1**
- ✅ **Require branches to be up to date before merging**
- ✅ Block force pushes

The third item is the important one: GitHub will refuse to merge a branch that is behind `main`,
so nobody can merge against a stale `main` whether or not they saw an announcement. Announcing
pushes is then only needed for the shared-area changes in section 3.
