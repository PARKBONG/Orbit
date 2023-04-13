import requests


def alert(env=0):
    token = "xoxb-5094106865095-5094120833959-GVRkWaaOScg2aGw1TpouDR1m"
    channel = "notification"

    if env :
        text = f"[Learning Done] : Workstation-{env}"
    else :
        text = "[Learning Done]"

    requests.post(url="https://slack.com/api/chat.postMessage",
                  headers={"Authorization": "Bearer " + token},
                  data={"channel": channel, "text": text})


if __name__ == "__main__":
    alert()
