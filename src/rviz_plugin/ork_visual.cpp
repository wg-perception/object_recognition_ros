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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "ork_visual.h"

namespace object_recognition_ros
{

// BEGIN_TUTORIAL
  OrkObjectVisual::OrkObjectVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
  {
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the Imu's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

    name_.reset(new rviz::MovableText("EMPTY"));
    name_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
    name_->setCharacterHeight(0.08);
    name_->showOnTop();
    name_->setVisible(false);
  }

  OrkObjectVisual::~OrkObjectVisual()
  {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode(frame_node_);
  }

  void
  OrkObjectVisual::setMessage(const object_recognition_msgs::RecognizedObject& object)
  {
    // Set the pose of the object
  axes_->setOrientation(
      Ogre::Quaternion(object.pose.pose.pose.orientation.w,
                       object.pose.pose.pose.orientation.x,
                       object.pose.pose.pose.orientation.y,
                       object.pose.pose.pose.orientation.z));
  axes_->setPosition(
      Ogre::Vector3(object.pose.pose.pose.position.x,
                    object.pose.pose.pose.position.y,
                    object.pose.pose.pose.position.z));

  // Set the name of the object
  name_->setCaption(object.type.type);
  //name_>setColor(color);
  name_->setGlobalTranslation(Ogre::Vector3(object.pose.pose.pose.position.x,
                                            object.pose.pose.pose.position.y,
                                            object.pose.pose.pose.position.z));
  }

// Position and orientation are passed through to the SceneNode.
  void
  OrkObjectVisual::setFramePosition(const Ogre::Vector3& position)
  {
    frame_node_->setPosition(position);
  }

  void
  OrkObjectVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
  {
    frame_node_->setOrientation(orientation);
  }

  // Color is passed through to the Arrow object.
  void
  OrkObjectVisual::setColor(float r, float g, float b, float a)
  {
    //acceleration_arrow_->setColor(r, g, b, a);
  }
// END_TUTORIAL

}// end namespace object_recognition_ros

