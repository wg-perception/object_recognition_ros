/*********************************************************************
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

#include <fstream>
#include <sstream>
#include <stdio.h>

#include <boost/scoped_ptr.hpp>

#include <geometric_shapes/shape_operations.h>

#include <object_recognition_ros/object_info_cache.h>

namespace object_recognition_ros {

ObjectInfoCache::ObjectInfoCache() {
}

void ObjectInfoCache::getInfoBase(const object_recognition_msgs::ObjectType & type, bool &is_cached,
                                  ObjectInfoPtr &object_info_ptr) {

  std::string object_hash = type.db + type.key;
  std::string mesh_resource;

  if (object_informations_.find(object_hash) != object_informations_.end()) {
    is_cached = true;
    object_info_ptr = object_informations_[object_hash];
    return;
  }
  is_cached = false;

  // Get the DB
  object_recognition_core::db::ObjectDbParameters db_params(type.db);
  std::string db_params_str;
  or_json::mValue db_val = db_params.raw();
  std::stringstream ss;
  or_json::write(db_val, ss);
  db_params_str = ss.str();
  if (db_loaded_.find(db_params_str) == db_loaded_.end()) {
    if (db_params.type()
        == object_recognition_core::db::ObjectDbParameters::NONCORE) {
      // If we're non-core, load the corresponding plugin
      if (!db_class_loader_)
        db_class_loader_.reset(
          new pluginlib::ClassLoader<object_recognition_core::db::ObjectDb>(
              "object_recognition_core", "object_recognition_core::db::ObjectDb"));
      try {
        db_loaded_[db_params_str] = db_class_loader_->createInstance(
            db_params.raw().at("type").get_str());
      } catch (pluginlib::PluginlibException& ex) {
        //handle the class failing to load
        ROS_ERROR("The plugin failed to load for some reason. Error: %s",
                  ex.what());
      }
      db_loaded_[db_params_str]->set_parameters(db_params);
    } else {
      db_loaded_[db_params_str] = db_params.generateDb();
    }
  }
  object_recognition_core::db::ObjectDbPtr db = db_loaded_[db_params_str];

  // Get information about the object
  try {
    object_info_ptr.reset(new object_recognition_core::prototypes::ObjectInfo(type.key, db));
  } catch (...) {
    ROS_ERROR("Cannot retrieve load mesh Object db not initialized");
  }
  object_info_ptr->load_fields_and_attachments();
  object_informations_[object_hash] = object_info_ptr;
}

ObjectInfoDiskCache::~ObjectInfoDiskCache() {
  // Clean up all the temporary files
  for (std::map<std::string, std::string>::iterator iter = mesh_files_.begin();
      iter != mesh_files_.end(); ++iter)
    std::remove(iter->second.c_str());
}

void ObjectInfoDiskCache::getInfo(
    const object_recognition_msgs::ObjectType & type,
    object_recognition_core::prototypes::ObjectInfo &object_info) {

  ObjectInfoPtr object_info_ptr;
  bool is_cached;
  getInfoBase(type, is_cached, object_info_ptr);
  if (is_cached) {
    object_info = *object_info_ptr;
    return;
  }

  // Fill the mesh
  if (!(object_info_ptr->has_field("mesh_uri")) && (object_info_ptr->has_attachment("mesh"))) {
    std::string mesh_resource;
    // If the full mesh is stored in the database, save it to a temporary file and use it as the mesh URI
    std::string file_name = std::string(std::tmpnam(0)) + ".stl";
    std::ofstream file;
    file.open(file_name.c_str(), std::ios::out | std::ios::binary);
    std::stringstream stream;
    object_info_ptr->get_attachment_stream("mesh", file);
    file.close();
    mesh_resource = std::string("file://") + file_name;
    // Keep track of the files to delete them later
    std::string object_hash = type.db + type.key;
    mesh_files_[object_hash] = file_name;

    object_info_ptr->set_field("mesh_uri", mesh_resource);
  }

  object_info = *object_info_ptr;
}

void ObjectInfoRamCache::getInfo(
    const object_recognition_msgs::ObjectType & type,
    object_recognition_msgs::ObjectInformation &info) {

  ObjectInfoPtr object_info_ptr;
  bool is_cached;
  getInfoBase(type, is_cached, object_info_ptr);

  // Fill the name
  if (object_info_ptr->has_field("name"))
    info.name = object_info_ptr->get_field<std::string>("name");

  boost::scoped_ptr<shapes::Mesh> mesh;

  if (object_info_ptr->has_attachment("mesh")) {
    std::stringstream mesh_stream(std::ios::in | std::ios::out | std::ios::binary);

    object_info_ptr->get_attachment_stream("mesh", mesh_stream);
    mesh_stream.seekg(0, std::ios::end);
    std::stringstream::pos_type size = mesh_stream.tellg();
    if (size <= 0)
      ROS_ERROR("Stored database mesh is empty for object key %s", type.key.c_str());
    else {
      char *buffer = new char[size];
      mesh_stream.seekg(0, std::ios::beg);
      mesh_stream.read(buffer, size);
      mesh.reset(shapes::createMeshFromBinary(buffer, size, "stl"));
      delete[] buffer;
      if (!mesh)
	ROS_ERROR("Unable to parse input mesh for object key %s", type.key.c_str());
      else
	ROS_DEBUG("Read mesh for object key '%s'", type.key.c_str());
    }
  } else if (object_info_ptr->has_field("mesh_uri")) {
    std::string mesh_uri = object_info_ptr->get_field<std::string>("mesh_uri");
    mesh.reset(shapes::createMeshFromResource(mesh_uri));
    if (!mesh)
      ROS_ERROR("Mesh resource '%s' not loaded", mesh_uri.c_str());
  } else
    ROS_WARN("No mesh information about object with key '%s'", type.key.c_str());

  if (mesh) {
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh.get(), shape_msg);
    info.ground_truth_mesh = boost::get<shape_msgs::Mesh>(shape_msg);
  }
}
}
