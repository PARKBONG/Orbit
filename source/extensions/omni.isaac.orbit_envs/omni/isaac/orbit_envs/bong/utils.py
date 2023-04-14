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


if __name__ == "__main__":
    slack_notification()
