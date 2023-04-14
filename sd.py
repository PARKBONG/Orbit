import subprocess


def get_hash():
    cmd = "git rev-parse HEAD"
    result = subprocess.run(cmd, shell=True, capture_output=True)
    hash = result.stdout.decode('utf-8').strip()
    return hash


git_hash = get_hash()
print(git_hash)

