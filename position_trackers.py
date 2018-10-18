
#!/usr/bin/env python
"""
The Pozyx ready to localize tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python

This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!

"""
from math import sqrt, atan2, pi
from time import sleep

from pypozyx import PozyxConstants, Coordinates, POZYX_SUCCESS, POZYX_ANCHOR_SEL_AUTO, version, \
    DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister
from pythonosc.udp_client import SimpleUDPClient

from pypozyx.tools.version_check import perform_latest_version_check


# IDs of the tags to position, add None to position the local tag as well.
TAG_IDS = [0x6746, 0x674B]


class MultitagPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, tag_ids, anchors, algorithm=PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                 dimension=PozyxConstants.DIMENSION_3D, height=1000):
        self.pozyx = pozyx

        self.osc_udp_client = SimpleUDPClient('localhost', 8888)
        self.engine_client = SimpleUDPClient('192.168.1.10', 2000)
        self.of_client = SimpleUDPClient('localhost', 9999)
        self.max_client = SimpleUDPClient('localhost', 7777)

        self.tag_ids = tag_ids
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height

        self.listeners = {t: Coordinates(0, 0, 0) for t in TAG_IDS if t is not None}
        self.tag_dist_traveled = {t: 0.0 for t in TAG_IDS if t is not None}

        self.prev_coordinates = Coordinates(0, 0, 0)
        self.distance = 0
        self.angle = 0
        self.middle = Coordinates(0, 0, 0)

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")
        print(" - System will manually calibrate the tags")
        print("")
        print(" - System will then auto start positioning")
        print("")
        if None in self.tag_ids:
            for device_id in self.tag_ids:
                self.pozyx.printDeviceInfo(device_id)
        else:
            for device_id in [None] + self.tag_ids:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")

        # plib = PozyxLib()
        # filter_data = FilterData()
        # self.pozyx.setPositioningFilterMovingAverage(10)
        # print(filter_data)

        self.pozyx.setRangingProtocolPrecision()
        print("Setting ranging protocol to precision")

        self.setAnchorsManual()

        self.printPublishAnchorConfiguration()

    def loop(self):
        """Performs positioning and prints the results."""
        for tag_id in self.tag_ids:
            position = Coordinates()
            status = self.pozyx.doPositioning(
                position, self.dimension, self.height, self.algorithm, remote_id=tag_id)
            if status == POZYX_SUCCESS:
                if tag_id:
                    room_position = get_4d_coordinates(position)
                    self.tag_dist_traveled[tag_id] = get_distance(room_position, self.listeners[tag_id])    # determines how far tag has traveled since the last update
                    self.listeners[tag_id] = room_position

                self.printPublishPosition(position, tag_id)
                self.publish_distance_traveled(tag_id)
            else:
                self.printPublishErrorCode("positioning", tag_id)

        self.distance, self.angle, self.middle = get_orientation_info(self.listeners[TAG_IDS[0]],
                                                                      self.listeners[TAG_IDS[1]])
        self.publish_orientation_info()

    def printPublishPosition(self, position, network_id):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        if network_id is None:
            network_id = 0
        s = "POS ID: {}, x(mm): {}, y(mm): {}, z(mm): {}".format("0x%0.4x" % network_id,
                                                                 position.x, position.y, position.z)
        print(s)
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, position.x, position.y, position.z])

            if network_id:
                position_to_send = self.listeners[network_id]

                args_to_send = [network_id, position_to_send.x, 2.5, position_to_send.y]

                self.max_client.send_message("/position/tag%d" % (TAG_IDS.index(network_id)+1), args_to_send)
                self.of_client.send_message("/position", args_to_send)

                if network_id == TAG_IDS[0]:
                    self.engine_client.send_message("/source21/position", [position_to_send.x, 2.5, position_to_send.y])
                if network_id == TAG_IDS[1]:
                    self.engine_client.send_message("/source22/position", [position_to_send.x, 2.5, position_to_send.y])

    def publish_orientation_info(self):
        self.max_client.send_message("/orientation/angle", self.angle)
        self.max_client.send_message("/orientation/distance", self.distance)
        self.max_client.send_message("/orientation/middle", [self.middle.x, 2.5, self.middle.y])

        self.of_client.send_message("/orientation/angle", [TAG_IDS[0], TAG_IDS[1], self.angle])
        self.of_client.send_message("/orientation/distance", [TAG_IDS[0], TAG_IDS[1], self.distance])
        self.of_client.send_message("/orientation/middle", [TAG_IDS[0], TAG_IDS[1], self.middle.x, 2.5, self.middle.y])

    def publish_distance_traveled(self, tag_id):
        self.max_client.send_message("/distance/tag%d" % (TAG_IDS.index(tag_id)+1), self.tag_dist_traveled[tag_id])

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for tag_id in self.tag_ids:
            status = self.pozyx.clearDevices(tag_id)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag_id)
            if len(anchors) > 4:
                status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(anchors),
                                                           remote_id=tag_id)
            # enable these if you want to save the configuration to the devices.
            if save_to_flash:
                self.pozyx.saveAnchorIds(tag_id)
                self.pozyx.saveRegisters([POZYX_ANCHOR_SEL_AUTO], tag_id)
            self.printPublishConfigurationResult(status, tag_id)

    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % tag_id)
        else:
            self.printPublishErrorCode("configuration", tag_id)

    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            print("Error %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error_%s" % operation, [network_id, error_code[0]])
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            print("Error % s, local error code %s" % (operation, str(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error_%s" % operation, [0, error_code[0]])

    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
                sleep(0.025)


X_OFFSET = -13330
Y_OFFSET = -6295


def get_4d_coordinates(position):
    return Coordinates(-(position.x + X_OFFSET) / 1000, -(position.y + Y_OFFSET) / 1000, position.z / 1000)


def get_distance(p1, p2):
    x_diff = (p2.x - p1.x)
    y_diff = (p2.y - p1.y)
    return sqrt(x_diff**2 + y_diff**2)


def get_orientation_info(p1, p2):
    dist = get_distance(p1, p2)
    middle = Coordinates((p1.x+p2.x)/2, (p1.y+p2.y)/2, (p1.z+p2.z)/2)
    angle = atan2((p2.y - p1.y), (p2.x - p1.x)) * 180 / pi

    return dist, angle, middle


if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself.
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # necessary data for calibration (SSI)
    anchors = [DeviceCoordinates(0x6764, 1, Coordinates(0, 0, 2352)),
               DeviceCoordinates(0x6756, 1, Coordinates(3761, 12591, 2958)),
               DeviceCoordinates(0x6727, 1, Coordinates(11879, 530, 2438)),
               DeviceCoordinates(0x6e19, 1, Coordinates(18858, 0, 3364)),
               DeviceCoordinates(0x6767, 1, Coordinates(18917, 12603, 2432)),
               DeviceCoordinates(0x6e17, 1, Coordinates(10979, 12061, 2438))]


    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING (must use 3d positioning for this)
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_3D
    # height of device, required in 2.5D positioning
    height = 1000

    pozyx = PozyxSerial(serial_port)

    r = MultitagPositioning(pozyx, TAG_IDS, anchors,
                            algorithm, dimension, height)
    r.setup()
    while True:
        r.loop()
