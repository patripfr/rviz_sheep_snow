/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, David Lu!!
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <rviz_sheep_snow/snow_display.h>

#include "rviz/display_context.h"
#include "rviz/default_plugin/markers/marker_base.h"
#include "rviz/default_plugin/marker_utils.h"
#include <visualization_msgs/Marker.h>
#include <OgreSceneNode.h>

namespace rviz_sheep_snow
{
SnowDisplay::SnowDisplay() : point_cloud_(nullptr)
{
  height_property_ = new rviz::FloatProperty("Height", 10.0, "Maximum Height", this, SLOT(updatePosition()));
  height_property_->setMin(0.0);

  width_property_ = new rviz::FloatProperty("Width", 10.0, "Total XY Dimension", this, SLOT(updatePosition()));
  width_property_->setMin(0.0);

  gravity_property_ = new rviz::FloatProperty("Gravity", 0.05, "Z motion per time step", this, SLOT(updatePosition()));
  wind_property_ = new rviz::FloatProperty("Wind", 0.02, "X motion per time step", this, SLOT(updatePosition()));
  jiggle_property_ = new rviz::FloatProperty("Jiggle", 0.03, "Magnitude of Jiggle", this, SLOT(updatePosition()));

  size_property_ = new rviz::IntProperty("Objects", 1000, "Number of snowflakes / sheeps", this, SLOT(updateSize()));
  size_property_->setMin(1);

  sheep_size_property_ = new rviz::FloatProperty("Sheep Size", 0.015, "Sheep Scaling", this, SLOT(updateSize()));
  sheep_size_property_->setMax(1000);
  sheep_size_property_->setMin(0.0000);

  snow_enabled_property_ = new rviz::BoolProperty("Enable Snow", false, "Enable Snow", this, SLOT(updateSize()));
  sheep_enabled_property_ = new rviz::BoolProperty("Enable Sheep", true, "Enable Sheep", this, SLOT(updateSize()));


  visualization_msgs::Marker marker;
  marker_message_.header.frame_id = "map";
  marker_message_.header.stamp = ros::Time();
  marker_message_.ns = "";
  marker_message_.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_message_.mesh_resource = "package://rviz_sheep_snow/meshes/sheep.dae";
  marker_message_.pose.orientation.x = 0.7071068;
  marker_message_.pose.orientation.y = 0.0;
  marker_message_.pose.orientation.z = 0.0;
  marker_message_.pose.orientation.w = 0.7071068;
  marker_message_.scale.x = 0.01;
  marker_message_.scale.y = 0.01;
  marker_message_.scale.z = 0.01;
  marker_message_.color.a = 0.0; // Don't forget to set the alpha!
  marker_message_.color.r = 0.0;
  marker_message_.color.g = 0.0;
  marker_message_.color.b = 0.0;
  marker_message_.action = visualization_msgs::Marker::MODIFY;
  marker_message_.mesh_use_embedded_materials = true;
}

void SnowDisplay::onInitialize()
{
  Display::onInitialize();
  if (!point_cloud_)
  {
    point_cloud_ = new rviz::PointCloud();
    scene_node_->attachObject(point_cloud_);
    point_cloud_->setAlpha(1.0);
  }

  updateSize();
}

void SnowDisplay::update(float wall_dt, float ros_dt)
{
  updatePosition();
}

void SnowDisplay::initializeXY(geometry_msgs::Point& pt) const
{
  pt.x = (randScale() - 0.5) * width_;
  pt.y = (randScale() - 0.5) * width_;
}

void SnowDisplay::updateSize()
{
  unsigned int size = static_cast<unsigned int>(size_property_->getInt());
  height_ = height_property_->getFloat();
  width_ = width_property_->getFloat();
  sheep_enabled_ = sheep_enabled_property_->getBool();
  snow_enabled_ = snow_enabled_property_->getBool();
  points_.resize(size);
  flakes_.resize(size);
  markers_.clear();
  id_vec_.clear();
  int i = 0;

  for (geometry_msgs::Point& point : points_)
  {
    initializeXY(point);
    point.z = randScale() * height_;
    if (sheep_enabled_) {
      MarkerBasePtr marker_ptr;
      marker_ptr.reset(rviz::createMarker(visualization_msgs::Marker::MESH_RESOURCE, &marker_display_, context_, scene_node_));
      marker_message_.id = i;
      marker_ptr->setMessage(marker_message_);
      MarkerID m_id("",i);
      markers_.insert(std::make_pair(m_id, marker_ptr));
      id_vec_.push_back(m_id);
      i++;
    }
  }
  updatePosition();
}

void SnowDisplay::updatePosition()
{
  double gravity = gravity_property_->getFloat();
  double wind = wind_property_->getFloat();
  double jiggle = jiggle_property_->getFloat();

  for (geometry_msgs::Point& point : points_)
  {
    point.x += wind;

    point.x += (randScale() - 0.5) * jiggle;
    point.y += (randScale() - 0.5) * jiggle;

    if (point.x >= width_ / 2)
    {
      point.x -= width_;
    }
    else if (point.x <= -width_ / 2)
    {
      point.x += width_;
    }

    point.z -= gravity;
    if (point.z <= 0.0)
    {
      initializeXY(point);
      point.z = height_;
    }
    else if (point.z >= height_)
    {
      point.z = 0.0;
    }
  }
  letItSnow();
}

void SnowDisplay::letItSnow()
{
  if (!point_cloud_)
  {
    return;
  }

  point_cloud_->clear();
  for (unsigned int i = 0; i < flakes_.size(); ++i)
  {
    if (snow_enabled_) {
      flakes_[i].position.x = points_[i].x;
      flakes_[i].position.y = points_[i].y;
      flakes_[i].position.z = points_[i].z;
      flakes_[i].setColor(1.0, 1.0, 1.0, 1.0);
    }

    if (sheep_enabled_) {
      marker_message_.id = i;
      marker_message_.pose.position.x = points_[i].x;
      marker_message_.pose.position.y = points_[i].y;
      marker_message_.pose.position.z = points_[i].z;
      marker_message_.scale.x = sheep_size_property_->getFloat();
      marker_message_.scale.y = sheep_size_property_->getFloat();
      marker_message_.scale.z = sheep_size_property_->getFloat();
      markers_[id_vec_[i]]->setMessage(marker_message_);
    }
  }

  if (snow_enabled_) {
    point_cloud_->addPoints(&flakes_.front(), flakes_.size());
  }
}
}  // namespace rviz_sheep_snow

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_sheep_snow::SnowDisplay, rviz::Display)
