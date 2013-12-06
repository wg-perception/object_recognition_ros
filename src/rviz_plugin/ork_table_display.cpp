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

#include <rviz/visualization_manager.h>

#include "ork_table_visual.h"

#include "ork_table_display.h"

namespace object_recognition_ros
{

OrkTableDisplay::OrkTableDisplay()
{
  // Create some display options
  do_display_hull_ = new rviz::BoolProperty("Hull", true, "Displays the hull or not.", this);
  do_display_bounding_box_ = new rviz::BoolProperty("Bounding Box", false,
                                                    "Displays the Bounding box or not.", this);
  do_display_top_ = new rviz::BoolProperty("Top", true, "Displays the top of the table or not.", this);
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void OrkTableDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

// Clear the visuals by deleting their objects.
void
OrkTableDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// This is our callback to handle an incoming message.
void
OrkTableDisplay::processMessage(const object_recognition_msgs::TableArrayConstPtr& msg)
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this message. If
  // it fails, we can't do anything else so we return.

  for (size_t i_msg = 0; i_msg < msg->tables.size(); ++i_msg) {
    const object_recognition_msgs::Table& table = msg->tables[i_msg];
    // Create a new visual for that message
    if (i_msg >= visuals_.size())
      visuals_.push_back(boost::shared_ptr<OrkTableVisual>(
        new OrkTableVisual(context_->getSceneManager(), scene_node_,
                           context_)));

    boost::shared_ptr<OrkTableVisual> &visual = visuals_[i_msg];
    visual->setMessage(table, do_display_hull_->getBool(), do_display_bounding_box_->getBool(),
                       do_display_top_->getBool());

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(table.header.frame_id, table.header.stamp, position, orientation)) {
      ROS_DEBUG(
        "Error transforming from frame '%s' to frame '%s'", table.header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);
  }
  visuals_.resize(msg->tables.size());
}
}  // end namespace object_recognition_ros

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(object_recognition_ros::OrkTableDisplay, rviz::Display)
