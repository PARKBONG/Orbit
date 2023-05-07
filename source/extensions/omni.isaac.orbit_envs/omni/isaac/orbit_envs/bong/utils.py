import requests
import subprocess


def slack_notification(env=0):
    token = "xoxb-5094106865095-5094120833959-i0TtABG8MR35g23NOwQYU80p"
    channel = "notification"

    if env :
        text = f"[Learning Done] : Workstation-{env}"
    else :
        text = "[Learning Done]"

    requests.post(url="https://slack.com/api/chat.postMessage",
                  headers={"Authorization": "Bearer " + token},
                  data={"channel": channel, "text": text})


def git_hash():
    cmd = "git rev-parse HEAD"
    result = subprocess.run(cmd, shell=True, capture_output=True)
    hash = str(result.stdout.decode('utf-8').strip())
    return hash

from shutil import copyfile

def save_files(log_dir, task_name, env_name, file_target = "bong_lift"):
    isaac_loc = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/"
    loc1 = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/bong/"
    loc2 = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/data/sb3/"
    copyfile(loc1 + task_name + "/" + file_target + "_cfg.py", isaac_loc + log_dir + "/" + file_target + "_cfg.py")
    copyfile(loc1 + task_name + "/" + file_target + "_env.py", isaac_loc + log_dir + "/" + file_target + "_env.py")
    copyfile(loc2 + env_name + ".yaml", isaac_loc + log_dir + "/" + env_name + ".yaml")

if __name__ == "__main__":
    slack_notification()

if __name__ == "__main__":
    slack_notification()