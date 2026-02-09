Run a Python script on the Jetson via SSH.

The user will provide the script path as an argument: $ARGUMENTS

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SCP the script to the Jetson home directory (`~/`)
3. SSH into the Jetson and run it with `python3` (NOT `python`)
4. Use interactive SSH via `expect` so we can see full output:
   - `spawn ssh <user>@<host>`
   - Send password when prompted
   - Wait for shell prompt
   - Send `python3 ~/<script_name>`
   - Wait for completion or error
5. Report the output

Use `expect` for SSH (no sshpass on macOS). Always use `python3` on Jetson.
