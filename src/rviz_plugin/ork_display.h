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

#ifndef ORK_DISPLAY_H
#define ORK_DISPLAY_H

#include <map>

#include <boost/foreach.hpp>

#include <pluginlib/class_loader.h>
#include <rviz/message_filter_display.h>

#include <object_recognition_core/db/db.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_ros/object_info_cache.h>

namespace Ogre
{
  class SceneNode;
}

namespace object_recognition_ros
{

  class OrkObjectVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// OrkObjectDisplay will show a 3D arrow showing the direction and magnitude
// of the IMU acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the Imu message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
  class OrkObjectDisplay: public rviz::MessageFilterDisplay<object_recognition_msgs::RecognizedObjectArray>
  {
    Q_OBJECT
  public:
    // Constructor.  pluginlib::ClassLoader creates instances by calling
    // the default constructor, so make sure you have one.
    OrkObjectDisplay();

    // Overrides of protected virtual functions from Display.  As much
    // as possible, when Displays are not enabled, they should not be
    // subscribed to incoming data and should not show anything in the
    // 3D view.  These functions are where these connections are made
    // and broken.
  protected:
    virtual void
    onInitialize();

    // A helper to clear this display back to the initial state.
    virtual void
    reset();

    // Function to handle an incoming ROS message.
  private:
    void
    processMessage(const object_recognition_msgs::RecognizedObjectArrayConstPtr& msg);

  /** Storage for the list of visuals */
  std::vector<boost::shared_ptr<OrkObjectVisual> > visuals_;
  /** Cache for al the info */
  ObjectInfoDiskCache info_cache_;
};

}

#endif // ORK_DISPLAY_H
