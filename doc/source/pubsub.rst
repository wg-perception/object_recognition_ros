:orphan:

.. _pubsub:

Publisher/Subscriber
####################

This package provides publishers and subscribers that are specific to ROS.


.. rubric:: Sources

.. program-output:: python -c "from object_recognition_core.utils.doc import config_yaml_for_ecto_cell; from object_recognition_ros.io.source import *; print config_yaml_for_ecto_cell(BagReader, 'BagReader') + '\n' + config_yaml_for_ecto_cell(RosKinect, 'RosKinect')"
   :shell:

.. rubric:: Sinks

.. program-output:: python -c "from object_recognition_core.utils.doc import config_yaml_for_ecto_cell; from object_recognition_ros.io.sink import Publisher; print config_yaml_for_ecto_cell(Publisher, 'Publisher')"
   :shell:
