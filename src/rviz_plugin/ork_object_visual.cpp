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

#include <OGRE/OgreCommon.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreVector3.h>

#include <rviz/display_context.h>
#include <rviz/display_factory.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "ork_object_visual.h"

namespace object_recognition_ros
{

// BEGIN_TUTORIAL
  OrkObjectVisual::OrkObjectVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                                   rviz::DisplayContext* display_context) : display_context_(display_context),
                                       mesh_entity_(0)
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
    object_node_ = frame_node_->createChildSceneNode();

  // Initialize the axes
  axes_.reset(new rviz::Axes(scene_manager_, object_node_));
  axes_->setScale(Ogre::Vector3(0.1, 0.1, 0.1));

  // Initialize the name
  name_.reset(new rviz::MovableText("EMPTY"));
  name_->setTextAlignment(rviz::MovableText::H_CENTER,
                          rviz::MovableText::V_CENTER);
  name_->setCharacterHeight(0.08);
  name_->showOnTop();
  name_->setColor(Ogre::ColourValue::White);
  name_->setVisible(false);

  object_node_->attachObject(name_.get());
}

  OrkObjectVisual::~OrkObjectVisual()
  {
    // Destroy the frame node since we don't need it anymore.
    if (mesh_entity_) {
      display_context_->getSceneManager()->destroyEntity(mesh_entity_);
      mesh_entity_ = 0;
    }
    scene_manager_->destroySceneNode(object_node_);
    scene_manager_->destroySceneNode(frame_node_);
  }

void
OrkObjectVisual::setMessage(const object_recognition_msgs::RecognizedObject& object, const std::string& name, const std::string& mesh_resource,
                            bool do_display_id, bool do_display_name, bool do_display_confidence)
{
    Ogre::Vector3 position(object.pose.pose.pose.position.x,
                        object.pose.pose.pose.position.y,
                        object.pose.pose.pose.position.z);
    object_node_->setOrientation(
        Ogre::Quaternion(object.pose.pose.pose.orientation.w,
                         object.pose.pose.pose.orientation.x,
                         object.pose.pose.pose.orientation.y,
                         object.pose.pose.pose.orientation.z));
    object_node_->setPosition(position);

  // Set the name of the object
  std::stringstream caption;
  if ((!object.type.key.empty()) && (do_display_id))
    caption << object.type.key << std::endl;
  if ((!name.empty()) && (do_display_name))
    caption << name << std::endl;

  // Set the caption of the object
  if (do_display_confidence)
    caption << object.confidence;

  if (caption.str().empty())
    name_->setVisible(false);
  else {
    name_->setCaption(caption.str());
    name_->setVisible(true);
    name_->setLocalTranslation(Ogre::Vector3(0.1, 0, 0));
  }

  if (!mesh_resource.empty()) {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "ork_mesh_resource_marker_" << count++;
    std::string id = ss.str();

    mesh_entity_ = display_context_->getSceneManager()->createEntity(
        id, mesh_resource);
    Ogre::MaterialPtr material = mesh_entity_->getSubEntity(0)->getMaterial();
    material->setCullingMode(Ogre::CULL_NONE);
    mesh_entity_->setMaterial(material);
    object_node_->attachObject(mesh_entity_);

    // In Ogre, mesh surface normals are not normalized if object is not
    // scaled.  This forces the surface normals to be renormalized by
    // invisibly tweaking the scale.
    Ogre::Vector3 scale(1, 1, 1.0001);
    frame_node_->setScale(scale);
  }
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
}    // end namespace object_recognition_ros
