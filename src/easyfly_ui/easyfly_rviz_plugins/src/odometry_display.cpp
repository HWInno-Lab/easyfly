// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "easyfly_rviz_plugins/odometry_display.hpp"
#include <vector>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <pluginlib/class_list_macros.hpp>
namespace easyfly_rviz_plugins
{
struct OdometryDisplay::NodeLifetime : Ogre::Node::Listener
{
  Ogre::SceneNode * node;
  Ogre::Node::Listener * previous;
  explicit NodeLifetime(Ogre::SceneNode * value)
  : node(value), previous(value->getListener()) {node->setListener(this);}
  ~NodeLifetime() override {if (node) {node->setListener(previous);}}
  void nodeUpdated(const Ogre::Node * value) override
  {if (previous) {previous->nodeUpdated(value);}}
  void nodeAttached(const Ogre::Node * value) override
  {if (previous) {previous->nodeAttached(value);}}
  void nodeDetached(const Ogre::Node * value) override
  {if (previous) {previous->nodeDetached(value);}}
  void nodeDestroyed(const Ogre::Node * value) override
  {node = nullptr; if (previous) {previous->nodeDestroyed(value);}}
};
OdometryDisplay::ObservedNodes OdometryDisplay::owned_nodes() const
{
  ObservedNodes nodes;
  if (scene_node_) {
    const auto visit = [&](const auto & self, Ogre::Node * parent) -> void {
        for (auto * child : parent->getChildren()) {
          nodes.push_back(std::make_unique<NodeLifetime>(static_cast<Ogre::SceneNode *>(child)));
          self(self, child);
        }
      };
    visit(visit, scene_node_);
  }
  return nodes;
}
void OdometryDisplay::remove_retired_parents(const ObservedNodes & owned)
{
  if (!scene_node_ || !scene_manager_) {return;}
  // Ogre nodes can be unnamed. Temporary listeners invalidate pointers on native
  // deletion, forward pre-existing listeners, then restore them on live nodes.
  // CovarianceVisual also leaves an empty, detached orientation root. Limit
  // cleanup to nodes captured in THIS display's tree before native retirement.
  for (const auto & observed : owned) {
    auto * child = observed->node;
    if (child && child->numChildren() == 0 && child->numAttachedObjects() == 0 &&
      (!child->getParent() || child->getParent() == scene_node_))
    {
      scene_manager_->destroySceneNode(child);
    }
  }
}
OdometryDisplay::~OdometryDisplay()
{
  if (scene_node_ && scene_manager_) {reset();}
}
void OdometryDisplay::update(float wall_dt, float ros_dt)
{
  const auto owned = owned_nodes();
  rviz_default_plugins::displays::OdometryDisplay::update(wall_dt, ros_dt);
  remove_retired_parents(owned);
}
void OdometryDisplay::reset()
{
  const auto owned = owned_nodes();
  rviz_default_plugins::displays::OdometryDisplay::reset();
  remove_retired_parents(owned);
}
}  // namespace easyfly_rviz_plugins
PLUGINLIB_EXPORT_CLASS(easyfly_rviz_plugins::OdometryDisplay, rviz_common::Display)
