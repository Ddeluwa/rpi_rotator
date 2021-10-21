import paramiko

try :
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)
    ssh.connect("x",port=2249, username="root", password="innonet160905")

    stdin, stdout, stderr = ssh.exec_command("pwd")
    lines = stdout.readlines()
    print(''.join(lines))

except Exception as err :
    print(err)
