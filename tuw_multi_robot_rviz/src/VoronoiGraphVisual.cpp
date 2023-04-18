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

//#include <OGRE/OgreVector3.h>
#include <OgreVector.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz_rendering/objects/line.hpp>
#include <tuw_multi_robot_msgs/msg/vertex.hpp>
#include <tuw_multi_robot_rviz/VoronoiGraphVisual.hpp>

#include <boost/format.hpp>

namespace tuw_multi_robot_rviz
{

VoronoiGraphVisual::VoronoiGraphVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
{
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the MarkerDetection's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

    // initialize global variables
    colorPath_ = Ogre::ColourValue(255, 0, 0);
    scalePoint_ = 0.1;
    scalePath_ = 1;
}

VoronoiGraphVisual::~VoronoiGraphVisual()
{
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode(frame_node_);
}

void VoronoiGraphVisual::setMessage(Graph::ConstSharedPtr msg)
{
    static double timeOld_;
    if (timeOld_ == msg->header.stamp.sec)
    {
        return;
    }
    timeOld_ = msg->header.stamp.sec;

    pathLine.resize(msg->vertices.size());
    crossingShape.resize(pathLine.size() * 2);
    seg_id_text.resize(msg->vertices.size());
    for (size_t i = 0; i < pathLine.size(); ++i)
    {
        tuw_multi_robot_msgs::msg::Vertex seg = msg->vertices[i];
        geometry_msgs::msg::Point p1 = seg.path.front();
        geometry_msgs::msg::Point p2 = seg.path.back();

        // 	Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( (*spline_)(i / (double)pointsNrPath_ )(2) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );

        Ogre::Quaternion rotation;
        rotation.x = 1;
        rotation.y = 0;
        rotation.z = 0;
        rotation.w = 0;
        Ogre::Quaternion rotation2;// = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::PI / 2.), Ogre::Vector3::UNIT_Y);

        pathLine[i].reset(new rviz_rendering::Line(scene_manager_, frame_node_));
        pathLine[i]->setColor(colorPath_);
        pathLine[i]->setPoints(Ogre::Vector3((p1.x)  + msg->origin.position.x, (p1.y)  + msg->origin.position.y, p1.z  + msg->origin.position.z), Ogre::Vector3((p2.x)  + msg->origin.position.x, (p2.y)  + msg->origin.position.y, p2.z  + msg->origin.position.z));
        pathLine[i]->setScale(Ogre::Vector3(scalePath_, scalePath_, scalePath_));

        crossingShape[2 * i].reset(new rviz_rendering::Shape(rviz_rendering::Shape::Sphere, scene_manager_, frame_node_));
        crossingShape[2 * i]->setColor(colorPath_);
        crossingShape[2 * i]->setPosition(Ogre::Vector3((p1.x)  + msg->origin.position.x, (p1.y)  + msg->origin.position.y, p1.z  + msg->origin.position.z));
        crossingShape[2 * i]->setOrientation(rotation * rotation2);
        crossingShape[2 * i]->setScale(Ogre::Vector3(scalePoint_, scalePoint_, scalePoint_));

        crossingShape[2 * i + 1].reset(new rviz_rendering::Shape(rviz_rendering::Shape::Sphere, scene_manager_, frame_node_));
        crossingShape[2 * i + 1]->setColor(colorPath_);
        crossingShape[2 * i + 1]->setPosition(Ogre::Vector3((p2.x)  + msg->origin.position.x, (p2.y)  + msg->origin.position.y, p2.z  + msg->origin.position.z));
        crossingShape[2 * i + 1]->setOrientation(rotation * rotation2);
        crossingShape[2 * i + 1]->setScale(Ogre::Vector3(scalePoint_, scalePoint_, scalePoint_));

        //auto current_pos = Ogre::Vector3((p1.x + msg->origin.position.x - p1.x + msg->origin.position.x), (p1.y - p2.y)  + msg->origin.position.y, (p1.z - p2.z)  + msg->origin.position.z);
        auto a = Ogre::Vector3(p1.x, p1.y, p1.z);
        auto b = Ogre::Vector3(p2.x, p2.y, p2.z);
        
        //auto current_pos = Ogre::Vector3((p1.x + msg->origin.position.x), (p1.y + msg->origin.position.y), (p1.z + msg->origin.position.z));
        auto current_pos = (a + (b-a)/2) + Ogre::Vector3(msg->origin.position.x, msg->origin.position.y, msg->origin.position.z);

        seg_id_text[i].reset(new TextVisual(scene_manager_, frame_node_, current_pos));
        seg_id_text[i]->setCharacterHeight(0.2);
    
        std::string capt = (boost::format("%d") % msg->vertices[i].id).str();
        seg_id_text[i]->setCaption(capt);
        seg_id_text[i]->setPosition(current_pos);// - Ogre::Vector3(0.25,0.25,0));
    }
}

// Position is passed through to the SceneNode.
void VoronoiGraphVisual::setFramePosition(const Ogre::Vector3 &position)
{
    frame_node_->setPosition(position);
}

// Orientation is passed through to the SceneNode.
void VoronoiGraphVisual::setFrameOrientation(const Ogre::Quaternion &orientation)
{
    frame_node_->setOrientation(orientation);
}

// Color is passed through to the Shape object.
void VoronoiGraphVisual::setPathColor(Ogre::ColourValue color)
{
    colorPath_ = color;
    for (auto &pathXYi : pathLine)
    {
        pathXYi->setColor(colorPath_);
    }
}

void VoronoiGraphVisual::setPointScale(float scale)
{
    scalePoint_ = scale;
    for (auto &pathThetai : crossingShape)
    {
        pathThetai->setScale(Ogre::Vector3(scalePoint_, scalePoint_, scalePoint_));
    }
}

void VoronoiGraphVisual::setPathScale(float scale)
{
    scalePath_ = scale;
    for (auto &pathThetai : pathLine)
    {
        pathThetai->setScale(Ogre::Vector3(scalePath_, scalePath_, scalePath_));
    }
}

} // namespace tuw_multi_robot_rviz
