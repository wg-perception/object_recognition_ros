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
#include <stdio.h>

#include <geometric_shapes/shape_operations.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/db_base.h>
#include <object_recognition_core/db/db_parameters.h>
#include <object_recognition_core/db/prototypes/object_info.h>
#include <object_recognition_msgs/GetObjectInformation.h>

static const std::string NODE_NAME = "object_info";

static bool onServiceRequest(
    object_recognition_msgs::GetObjectInformation::Request &req,
    object_recognition_msgs::GetObjectInformation::Response &res) {

  // Get the DB
  object_recognition_core::db::ObjectDbPtr db;
  object_recognition_core::db::ObjectDbParameters db_params(req.type.db);
  if (db_params.type()
      == object_recognition_core::db::ObjectDbParameters::NONCORE) {
    // If we're non-core, load the corresponding plugin
    try {
      pluginlib::ClassLoader<object_recognition_core::db::ObjectDb> db_class_loader(
          "object_recognition_core", "object_recognition_core::db::ObjectDb");
      db = db_class_loader.createInstance(
          db_params.raw().at("type").get_str());
    } catch (pluginlib::PluginlibException& ex) {
      //handle the class failing to load
      ROS_ERROR("The plugin failed to load for some reason. Error: %s",
                ex.what());
    }
  } else {
    db = db_params.generateDb();
  }

  // Get information about the object
  object_recognition_core::prototypes::ObjectInfo object_info;
  try {
    object_info = object_recognition_core::prototypes::ObjectInfo(
        req.type.key, db);
  } catch (...) {
    ROS_ERROR("Cannot retrieve load mesh Object db not initialized");
  }
  object_info.load_fields_and_attachments();

  // Fill the name
  if (object_info.has_field("name"))
    res.information.name = object_info.get_field<std::string>("name");

  // Use the mesh information
  std::string mesh_resource;
  std::string mesh_file_name;
  if (object_info.has_field("mesh_uri")) {
    mesh_resource = object_info.get_field<std::string>("mesh_uri");
  } else if (object_info.has_attachment("mesh")) {
    // If the full mesh is stored in the object, save it to a temporary file and use it as the mesh URI
    mesh_file_name = std::string(std::tmpnam(0)) + ".stl";
    std::ofstream file(mesh_file_name.c_str(), std::ios::out | std::ios::binary);
    object_info.get_attachment_stream("mesh", file);
    file.close();
    mesh_resource = std::string("file://") + mesh_file_name;
  }

  if (!mesh_resource.empty()) {
    shapes::ShapeMsg shape_msg;
    boost::scoped_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(mesh_resource));
    if (mesh)
    {
      shapes::constructMsgFromShape(mesh.get(), shape_msg);
      res.information.ground_truth_mesh = boost::get<shape_msgs::Mesh>(shape_msg);
    }
    else
      ROS_ERROR_STREAM("Unable to construct shape message from " << mesh_resource);
  }
  else
    ROS_ERROR("Could not retrieve the mesh");

  // Delete the temporary file
  if (!mesh_file_name.empty())
    std::remove(mesh_file_name.c_str());

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::ServiceServer info_service = nh.advertiseService("get_object_info",
                                                        &onServiceRequest);
  ros::waitForShutdown();

  return 0;
}
