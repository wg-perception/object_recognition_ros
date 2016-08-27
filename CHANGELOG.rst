0.3.7 (2016-08-27)
------------------
* fix compilation on Jessie and Willy
* no C++11 for indigo and jade
* Contributors: Vincent Rabaud

0.3.6 (2016-08-27)
------------------
* do not risk anything bellow kinetic
* build on Kinetic
* dox fix (fixes `#18 <https://github.com/wg-perception/object_recognition_ros/issues/18>`_)
* try a longer wait to get the buildfarm working
* convert tests into proper rostests
* clean extensions
* Contributors: Vincent Rabaud

0.3.5 (2014-11-20)
------------------
* remove useless dependency on tf
* Merge pull request `#17 <https://github.com/wg-perception/object_recognition_ros/issues/17>`_ from v4hn/boost-scoped-ptr
  add missing include of scoped_ptr
* add missing include of scoped_ptr
  This becomes necessary with boost 1.57
* export the proper library
* Contributors: Michael Görner, Vincent Rabaud

0.3.4 (2014-09-18)
------------------
* move RViz documentation to object_recognition_ros_visualization
* simplify package.xml
* add proper dependencies
* split ork_ros into ork_ros and ork_ros_visualization
* Contributors: Ha Dang, Vincent Rabaud

0.3.3 (2014-08-14)
------------------
* Added try/catch in rviz plugin to prevent crash
* Contributors: Dave Hershberger, Sammy Pfeiffer

0.3.2 (2014-04-13)
------------------
* compile under Indigo
* compile under Indigo
* Merge pull request `#10 <https://github.com/wg-perception/object_recognition_ros/issues/10>`_ from v4hn/pose_result-no-db-fix
  don't segfault without a valid database entry
* comply to the new API
* fixes `#12 <https://github.com/wg-perception/object_recognition_ros/issues/12>`_
* Merge pull request `#11 <https://github.com/wg-perception/object_recognition_ros/issues/11>`_ from v4hn/table-nan-no-ogre-exception
  don't throw ogre exception when receiving invalid table
* don't throw ogre exception when receiving invalid table
  Although it should not happen, the plugin should still
  verify it's input messages before passing them on to OGRE.
  Otherwise rviz crashes:
  rviz: /build/buildd/ogre-1.7.4/OgreMain/src/OgreNode.cpp:413: virtual void
  Ogre::Node::setPosition(const Ogre::Vector3&): Assertion `!pos.isNaN() &&
  "Invalid vector supplied as parameter"' failed.
  Aborted (core dumped)
* don't segfault without a valid database entry
* Contributors: Vincent Rabaud, v4hn

0.3.1 (2013-12-18  21:12:06 +0100)
----------------------------------
- fix disappearing mesh
- add options for displaying text in plugins

0.3.0 (2013-12-08  19:12:06 +0100)
----------------------------------
- add a plugin to display tables
- drop Fuerte support
- remove the OpenCV dependency
- remove the PCL dependency

0.2.4 (2013-09-26 00:13:06 +0100)
---------------------------------
- fixes #7

0.2.3 (2013-08-28 19:21:21 +0100)
---------------------------------
- fix dependencies with PCL
