import ntcore
import time
class NetworkTable:
    def __init__(self, simulation=True):
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.nt.startClient4("note")
        if simulation == False:
            self.nt.setServerTeam(4183)
        else:
            self.nt.setServer("localhost")

        self.table = self.nt.getTable("vision")


        self.xTopic = self.table.getDoubleTopic("x")
        self.yTopic = self.table.getDoubleTopic("y")
        self.detectedTopic = self.table.getBooleanTopic("detected")

        self.xPublisher = self.xTopic.publish()
        self.yPublisher = self.yTopic.publish()
        self.detectedPublisher = self.detectedTopic.publish()

    def periodic(self, x, y, detected):
        # publish a value with current timestamp
        self.xPublisher.set(x, 0)  # 0 = use current time
        self.yPublisher.set(y, 0)
        self.detectedPublisher.set(detected, 0)

    def close(self):
        # stop publishing
        self.xPublisher.close()
        self.xTopic.close()
        self.yPublisher.close()
        self.yTopic.close()

if __name__ == "__main__":
    nt = NetworkTable(True)
    while True:
        time.sleep(0.5);
        nt.periodic(500, 100, True)