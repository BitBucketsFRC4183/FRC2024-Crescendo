import ntcore

class NetworkTable:
    def __init__(self, dblTopic: ntcore.DoubleTopic):

        # start publishing
        self.dblPub = dblTopic.publish()

        # specify publish options
        self.dblPub = dblTopic.publish(ntcore.PubSubOptions(keepDuplicates=True))

        # set initial properties and custom type string
        self.dblPub = dblTopic.publishEx("double", '{"xCoor": 0, "yCoor": 0}')

    def periodic(self, x, y):
        # publish a default value
        self.dblPub.setDefault(0.0)

        # publish a value with current timestamp
        self.dblPub.set(2.0, 0)  # 0 = use current time

    def close(self):
        # stop publishing
        self.dblPub.close()