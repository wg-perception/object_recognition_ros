"""
Module defining several outputs for the object recognition pipeline
"""

from ecto import BlackBoxCellInfo, BlackBoxForward
from object_recognition_core.boost.interface import ObjectDbParameters
from object_recognition_core.io.sink import SinkBase
from object_recognition_ros import init_ros
from object_recognition_ros.ecto_cells.ecto_object_recognition_msgs import Publisher_RecognizedObjectArray
from object_recognition_ros.ecto_cells.io_ros import MsgAssembler
import ecto
import ecto_ros.ecto_geometry_msgs as ecto_geometry_msgs
import ecto_ros.ecto_std_msgs as ecto_std_msgs

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray
StringPub = ecto_std_msgs.Publisher_String

########################################################################################################################

class Publisher(ecto.BlackBox, SinkBase):
    """Class publishing the different results of object recognition as ROS topics
    """
    def __init__(self, *args, **kwargs):
        init_ros()
        ecto.BlackBox.__init__(self, *args, **kwargs)
        SinkBase.__init__(self)

    @staticmethod
    def declare_cells(_p):
        return {'msg_assembler': BlackBoxCellInfo(MsgAssembler)}

    @staticmethod
    def declare_direct_params(p):
        p.declare('markers_topic', 'The ROS topic to use for the marker array.', 'markers')
        p.declare('pose_topic', 'The ROS topic to use for the pose array.', 'poses')
        p.declare('object_ids_topic', 'The ROS topic to use for the object meta info string', 'object_ids')
        p.declare('recognized_object_array_topic', 'The ROS topic to use for the recognized object', 'recognized_object_array')
        p.declare('latched', 'Determines if the topics will be latched.', True)
        p.declare('db_params', 'The DB parameters', ObjectDbParameters({}))

    @staticmethod
    def declare_forwards(_p):
        p = {'msg_assembler': [BlackBoxForward('publish_clusters')]}
        i = {'msg_assembler': 'all'}

        return (p,i,{})

    def configure(self, p, _i, _o):
        self._recognized_object_array = Publisher_RecognizedObjectArray(topic_name=p.recognized_object_array_topic, latched=p.latched)

    def connections(self, p):
        # connect to a publishing cell
        connections = [ self.msg_assembler['msg'] >> self._recognized_object_array['input']]

        return connections
