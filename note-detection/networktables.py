import ntcore

class NetworkTable:
    def __init__(self):

        self.xTopic = ntcore.NetworkTableInstance.getDefault().getDoubleTopic("vision/x")
        self.yTopic = ntcore.NetworkTableInstance.getDefault().getDoubleTopic("vision/y")
        self.detectedTopic = ntcore.NetworkTableInstance.getDefault().getBooleanTopic("vision/detected")

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