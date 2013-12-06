/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ORK_TABLE_VISUAL_H
#define ORK_TABLE_VISUAL_H

#include <object_recognition_msgs/Table.h>

namespace Ogre
{
class Quaternion;
class SceneManager;
class SceneNode;
class Vector3;
}

namespace rviz
{
class Arrow;
class BillboardLine;
}

namespace object_recognition_ros
{
// Declare the visual class for this display.
//
// Each instance of OrkTableVisual represents an object with its mesh and pose
class OrkTableVisual
{
public:
  /** Constructor.  Creates the visual stuff and puts it into the
   * scene, but in an unconfigured state.
   *
   * @param display The display that calls those visuals
   */
  OrkTableVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz::DisplayContext* display);

  // Destructor.  Removes the visual stuff from the scene.
  virtual
  ~OrkTableVisual();

  /** Configure the visual to show the data in the message.
   * @param table The table message
   */
  void
  setMessage(const object_recognition_msgs::Table& table, bool do_display_hull, bool do_display_bounding_box,
             bool do_display_top);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way ImuVisual is only
  // responsible for visualization.
  void
  setFramePosition(const Ogre::Vector3& position);
  void
  setFrameOrientation(const Ogre::Quaternion& orientation);
private:
  /** The convex hull of the table */
  boost::shared_ptr<rviz::BillboardLine> convex_hull_;

  /** The bounding box of the table */
  boost::shared_ptr<rviz::BillboardLine> bounding_box_;

  /** The pose of the table */
  boost::shared_ptr<rviz::Arrow> arrow_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Object message header.
  Ogre::SceneNode* frame_node_;
  Ogre::SceneNode* object_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};

}// end namespace rviz_plugin_tutorials

#endif // ORK_TABLE_VISUAL_H
