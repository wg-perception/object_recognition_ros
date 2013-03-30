/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ORK_OBJECT_INFO_CACHE_H_
#define ORK_OBJECT_INFO_CACHE_H_

#include <string>

#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>

#include <object_recognition_core/db/prototypes/object_info.h>
#include <object_recognition_msgs/ObjectInformation.h>
#include <object_recognition_msgs/ObjectType.h>

namespace object_recognition_ros {

class ObjectInfoCache {
 public:
  ObjectInfoCache();

 protected:
  typedef boost::shared_ptr<object_recognition_core::prototypes::ObjectInfo> ObjectInfoPtr;
  /**
   *
   * @param type the type of the object
   * @param is_cached true if the object info is already in the cache
   * @param info the info of the object
   */
  void
  getInfoBase(const object_recognition_msgs::ObjectType & type, bool &is_cached,
              ObjectInfoPtr &object_info_ptr);

 private:
  /** Loader for the custom DB classes */
  boost::shared_ptr<
      pluginlib::ClassLoader<object_recognition_core::db::ObjectDb> > db_class_loader_;
  /** Temporary storage for the loaded DB's (for reusability) */
  std::map<std::string, object_recognition_core::db::ObjectDbPtr> db_loaded_;
  /** Keep track of the RViz resources containing the meshes retrieved for the DB */
  std::map<std::string, ObjectInfoPtr> object_informations_;
};

class ObjectInfoDiskCache : public ObjectInfoCache {
 public:
  ~ObjectInfoDiskCache();

  void
  getInfo(const object_recognition_msgs::ObjectType & type,
          object_recognition_core::prototypes::ObjectInfo &info);

 private:
  /** Keep track of the files loaded from the DB and stored locally in a tmp file */
  std::map<std::string, std::string> mesh_files_;
};

class ObjectInfoRamCache : public ObjectInfoCache {
 public:
  void
  getInfo(const object_recognition_msgs::ObjectType & type,
          object_recognition_msgs::ObjectInformation &info);

 private:
  /** Keep track of the RViz resources containing the meshes retrieved for the DB */
  std::map<std::string, object_recognition_msgs::ObjectInformation> object_informations_;
};
}

#endif /* ORK_OBJECT_INFO_CACHE_H_ */
