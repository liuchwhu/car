Deploy the pilotnano package and scripts to the Jetson.

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SCP the following to the Jetson:
   - `scripts/` → `~/scripts/`
   - `src/pilotnano/` → `~/pilotnano/` (the package source)
   - `configs/` → `~/configs/`
3. SSH in and verify the files landed correctly
4. Report what was deployed

Use `expect` for SSH/SCP (no sshpass on macOS). Use `scp -r` for directories.
