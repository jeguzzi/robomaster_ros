from typing import TYPE_CHECKING, List, Optional, Tuple, Union, cast

import robomaster.robot
import robomaster.vision
import robomaster_msgs.msg

if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module

ROI = Tuple[float, float, float, float]
Line = Tuple[float, float, float, float]
ROI_ID = Tuple[float, float, float, float, str]

DetectionData = Union[int, Line, ROI, ROI_ID]
Detection = Tuple[int, int, List[DetectionData]]


def vision_data_info(self: robomaster.vision.VisionPushEvent) -> Detection:
    return self._type, self._status, self._rect_info


robomaster.vision.VisionPushEvent.data_info = vision_data_info


# TODO(jerome) : tentative
def roi(x: float, y: float, w: float,
        h: float) -> robomaster_msgs.msg.RegionOfInterest:
    return robomaster_msgs.msg.RegionOfInterest(x_offset=x,
                                                y_offset=y,
                                                width=w,
                                                height=h)


class Vision(Module):

    def __init__(self, robot: robomaster.robot.Robot,
                 node: 'RoboMasterROS') -> None:
        self.api = robot.vision
        self.node = node
        self.clock = node.get_clock()
        # DONE(jerome): expose as params or service
        self.vision_targets: List[str] = node.declare_parameter(
            'vision.targets', ["marker:red", "robot"]).value
        # Alternatively, I could use vision_msgs/Detection2DArray
        # But this requires a mapping between id and RM class types
        self.vision_pub = node.create_publisher(robomaster_msgs.msg.Detection,
                                                "vision", 10)
        self.frame_id = node.tf_frame('camera_optical_link')
        node.get_logger().info(
            f"Enabling vision to detect {self.vision_targets}")
        for name in self.vision_targets:
            color: Optional[str]
            try:
                name, color = name.split(":")
            except ValueError:
                color = None
            self.api.sub_detect_info(name=name,
                                     color=color,
                                     callback=self.got_vision)

    def stop(self) -> None:
        if self.node.connected:
            for name in self.vision_targets:
                try:
                    self.api.unsub_detect_info(name=name)
                except TypeError:
                    pass

    def abort(self) -> None:
        pass

    def detection_msg(self) -> robomaster_msgs.msg.Detection:
        msg = robomaster_msgs.msg.Detection()
        msg.header.stamp = self.clock.now().to_msg()
        msg.header.frame_id = self.frame_id
        return msg

    def has_detected_people(self, values: List[ROI]) -> None:
        msg = self.detection_msg()
        for (x, y, w, h) in values:
            msg.people.append(
                robomaster_msgs.msg.DetectedPerson(roi=roi(x, y, w, h)))
        self.vision_pub.publish(msg)

    def has_detected_robots(self, values: List[ROI]) -> None:
        msg = self.detection_msg()
        for (x, y, w, h) in values:
            msg.robots.append(
                robomaster_msgs.msg.DetectedRobot(roi=roi(x, y, w, h)))
        self.vision_pub.publish(msg)

    def has_detected_markers(self, values: List[ROI_ID]) -> None:
        msg = self.detection_msg()
        for (x, y, w, h, kind) in values:
            msg.markers.append(
                robomaster_msgs.msg.DetectedMarker(kind=kind,
                                                   roi=roi(x, y, w, h)))
        self.vision_pub.publish(msg)

    def has_detected_gestures(self, values: List[ROI_ID]) -> None:
        msg = self.detection_msg()
        for (x, y, w, h, kind) in values:
            msg.gestures.append(
                robomaster_msgs.msg.DetectedGesture(kind=kind,
                                                    roi=roi(x, y, w, h)))
        self.vision_pub.publish(msg)

    def has_detected_lines(self, values: List[Line]) -> None:
        msg = self.detection_msg()
        for (x, y, curvature, angle) in values:
            msg.lines.append(
                robomaster_msgs.msg.DetectedLine(x=x,
                                                 y=y,
                                                 curvature=curvature,
                                                 angle=angle))
        self.vision_pub.publish(msg)

    def got_vision(self, msg: Detection) -> None:
        kind, _, data = msg
        if kind == 1:
            self.has_detected_people(cast(List[ROI], data))
        elif kind == 2:
            self.has_detected_gestures(cast(List[ROI_ID], data))
        elif kind == 4:
            self.has_detected_lines(cast(List[Line], data[1:]))
        elif kind == 5:
            self.has_detected_markers(cast(List[ROI_ID], data))
        elif kind == 7:
            self.has_detected_robots(cast(List[ROI], data))
