.. _msgs:

Messages
########


Object Definition
*****************

An object is uniquely defined by a type and a database in ``ObjectId.msg``:

.. program-output:: rosmsg show -r object_recognition_msgs/ObjectType.msg

More information is also stored by ``ORK`` in the database. This information can be retrieved in
``ObjectInformation.msg``:

.. program-output:: rosmsg show -r object_recognition_msgs/ObjectInformation.msg

Recognized Objects
******************

When the objects are recognized, an array of recognized objects is published in ``RecognizedObjectArray.msg``:

.. program-output:: rosmsg show -r object_recognition_msgs/RecognizedObjectArray.msg

which contains several ``RecognizedObject.msg``:

.. program-output:: rosmsg show -r object_recognition_msgs/RecognizedObject.msg

Table
*****

There is that object that is pretty useful to recognize and it's called a table. When found with the
:ref:`Tabletop Detector <orktabletop:tabletop>` it publishes those two messages:

.. program-output:: rosmsg show -r object_recognition_msgs/Table.msg

.. program-output:: rosmsg show -r object_recognition_msgs/TableArray.msg
