:orphan:

.. _actionlib:

Actionlib Server
################

The Actionlib server is defined at :ref:`Actionlib Server <orkmsgs:action>`.


To run it, you just need to start it with:


.. code-block:: sh
   
      rosrun object_recognition_server server -c whatever_config_file.ork

You can also test it using the client:

.. code-block:: sh

      rosrun object_recognition_server client

And that should display the received message.


For testing, you can even run the server with a test pipeline:

.. code-block:: sh

      rosrun object_recognition_server server -c `rospack find object_recognition_ros`/conf/detection.test.ros.ork

