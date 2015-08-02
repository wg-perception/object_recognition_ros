:orphan:

.. _actionlib:

Actionlib Server
################

The Actionlib server helps you retrieve the identified objects in the current snapshot.
It is defined as follows:

.. literalinclude:: ../../../ork_msgs/action/ObjectRecognition.action

To run it, you just need to start it with:


.. code-block:: sh
   
      rosrun object_recognition_ros server -c whatever_config_file.ork

You can also test it using the client:

.. code-block:: sh

      rosrun object_recognition_ros client

And that should display the received message.


For testing, you can even run the server with a test pipeline:

.. code-block:: sh

      rosrun object_recognition_server server -c `rospack find object_recognition_ros`/conf/detection.test.ros.ork

.. _srv:

Service
#######

There is a service to get object information: just query it and it will retrieve anything
that is known about the object. Its definition is as follows:

.. program-output:: rossrv show -r object_recognition_msgs/GetObjectInformation.srv
