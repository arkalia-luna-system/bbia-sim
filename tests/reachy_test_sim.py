from reachy_sdk import ReachySDK


def test_connection():
    reachy = ReachySDK(host="localhost")
    print("🤖 Reachy simulation connectée :", reachy)


if __name__ == "__main__":
    test_connection()
