Run a shell command on the Jetson via SSH and show the output.

The user will provide the command as an argument: $ARGUMENTS

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SSH into the Jetson using `expect` and run the command
3. Report the output

Use `expect` for SSH (no sshpass on macOS). Example:
```
expect -c '
set timeout 30
spawn ssh <user>@<host>
expect "password:" { send "<pass>\r" }
expect "\\$"
send "<command>\r"
expect "\\$"
send "exit\r"
expect eof
'
```
