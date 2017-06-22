/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#include "ig_active_reconstruction/view_space.hpp"
#include <fstream>
#include <stdexcept>

namespace ig_active_reconstruction
{
  
namespace views
{

ViewSpace::ViewSpace()
{
  
}

std::vector<View, Eigen::aligned_allocator<View> > ViewSpace::getViewSpace()
{
  return view_space_;
}

void ViewSpace::getGoodViewSpace( IdSet& out, bool ignore_visited )
{
  for( auto& pair: views_index_map_ )
  {
    View& view = pair.second;
    if( view.reachable() && (view.timesVisited()==0||!ignore_visited) && !view.bad() )
    {
      out.push_back(view.index());
    }
  }
}

View ViewSpace::getView( View::IdType index )
{
  try
  {
    return views_index_map_.at(index);
  }
  catch(...)
  {
    throw std::out_of_range("ViewSpace::getView: the given index is out of range");
  }
}

bool ViewSpace::deleteView( View::IdType index )
{
  decltype(views_index_map_)::iterator it = views_index_map_.begin();
  decltype(views_index_map_)::iterator end = views_index_map_.end();
  
  for( ; it!=end; ++it )
  {
    if( it->second.index()==index )
    {
      views_index_map_.erase(it);
      recalculateIndexMap();
      return true;
    }
  }
  return false;
}

bool ViewSpace::deleteViews( std::vector<View::IdType>& index_set )
{
  bool found = true;
  for( View::IdType& id: index_set )
  {
    found = found && deleteView(id);
  }
  
  return found;
}

unsigned int ViewSpace::timesVisited( View::IdType index )
{
  try
  {
    return static_cast<View&>(views_index_map_.at(index)).timesVisited();
  }
  catch(...)
  {
    return 0;
  }
}

void ViewSpace::setBad( View::IdType index )
{
  try
  {
    static_cast<View&>(views_index_map_.at(index)).bad()=true;
    return;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setGood( View::IdType index )
{
  try
  {
    static_cast<View&>(views_index_map_.at(index)).bad()=false;
    return;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setVisited( View::IdType index )
{
  try
  {
    static_cast<View&>(views_index_map_.at(index)).timesVisited() += 1;
    return;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setUnReachable( View::IdType index )
{
  try
  {
    static_cast<View&>(views_index_map_.at(index)).reachable() = false;
    return;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setReachable( View::IdType index )
{
  try
  {
    static_cast<View&>(views_index_map_.at(index)).reachable() = true;
    return;
  }
  catch(...)
  {
    return;
  }
}


void ViewSpace::push_back( View new_vp )
{
  //view_space_.push_back(new_vp);
  //View& view_ref = view_space_.back();
  views_index_map_[new_vp.index()] = new_vp;//std::reference_wrapper<View>(view_ref);
}

View ViewSpace::getAClosestNeighbour( View& _view )
{
  if( views_index_map_.empty() )
    throw std::runtime_error("ViewSpace::getAClosestNeighbour::Cannot find a closest neighbour since the view space is empty.");
    
  Eigen::Vector3d probe = _view.pose().position;
  View closest = (views_index_map_.begin())->second;//view_space_[0];
  Eigen::Vector3d dist_vec = probe - closest.pose().position;
  double distance = dist_vec.norm();
  
  for( auto& pair: views_index_map_ )
  {
    View& view = pair.second;
    dist_vec = probe - view.pose().position;
    double norm = dist_vec.norm();
    if( norm<distance )
    {
      distance=norm;
      closest = view;
    }
  }
  return closest;
}

unsigned int ViewSpace::size()
{
  return views_index_map_.size();
}

void ViewSpace::getViewsInRange( View& _reference_view, double _distance, std::vector<View, Eigen::aligned_allocator<View> >& _sub_space )
{
  for( auto& pair: views_index_map_ )
  {
    View& view = pair.second;
    Eigen::Vector3d dist_vec = view.pose().position - _reference_view.pose().position;
    if( dist_vec.norm()<=_distance )
    {
      _sub_space.push_back(view);
    }
  }
}

static void writePoseToFile( std::ofstream& out, const movements::Pose& pose )
{
  out << pose.position.x();
  out << " " << pose.position.y();
  out << " " << pose.position.z();
  out << " " << pose.orientation.x();
  out << " " << pose.orientation.y();
  out << " " << pose.orientation.z();
  out << " " << pose.orientation.w();
}

/*
void ViewSpace::saveToFile( std::string _filename )
{
  std::ofstream out( _filename, std::ofstream::trunc );
  
  out<<views_index_map_.size();
  
  //for( unsigned int i=0; i<view_space_.size(); ++i )
  for( auto& pair: views_index_map_)
  {
    View& view = pair.second;
    out<<"\n";
    movements::Pose pose = view.pose();//view_space_[i].pose();
    writePoseToFile(out, pose);
  }
    
  out.close();
}
*/

void ViewSpace::saveToFile( std::string _filename )
{
  std::ofstream out( _filename, std::ofstream::trunc );

  out<<views_index_map_.size()<<"\n";

  // Writing the additional fields size and names
  if (views_index_map_.size() == 0)
  {
    // The viewspace is empty (we don't care if there are additional fields)
    out << "0\n";
  }
  else
  {
    std::vector<std::string>& additional_fields_names = views_index_map_[0].additionalFieldsNames();
    out << additional_fields_names.size();
    for( unsigned int i = 0; i < additional_fields_names.size(); i++ )
    {
      out << " " << additional_fields_names[i];
    }
    out << "\n";
  }

  //for( unsigned int i=0; i<view_space_.size(); ++i )
  for( auto& pair: views_index_map_)
  {
    View& view = pair.second;
    out<<"\n";

    movements::Pose pose = view.pose();//view_space_[i].pose();
    writePoseToFile(out, pose);

    std::vector<double>& additional_fields_values = view.additionalFieldsValues();
    for( unsigned int i = 0; i < additional_fields_values.size(); i++ )
    {
      out << " " << additional_fields_values[i];
    }
  }

  out.close();
}

static bool readPoseFromFile( std::ifstream& in, movements::Pose& pose )
{
  bool success = true;

  success = success && ( in>>pose.position.x() );
  success = success && ( in>>pose.position.y() );
  success = success && ( in>>pose.position.z() );
  success = success && ( in>>pose.orientation.x() );
  success = success && ( in>>pose.orientation.y() );
  success = success && ( in>>pose.orientation.z() );
  success = success && ( in>>pose.orientation.w() );

  return success;
}

/*
void ViewSpace::loadFromFile( std::string _filename )
{
  std::ifstream in(_filename, std::ifstream::in);
  
  unsigned int nr_of_views;
  bool success = (in >> nr_of_views);
  
  if(!success)
    return;
  
  for( unsigned int i=0; i<nr_of_views; ++i )
  {
    View new_pose;
    success = readPoseFromFile(in, new_pose.pose());
    
    if(!success)
      return;
    
    new_pose.bad() = false;
    new_pose.reachable() = true;
    new_pose.timesVisited() = 0;
    
    //view_space_.push_back(new_pose);
    views_index_map_[new_pose.index()] = new_pose;//view_space_.back();
  }
}
*/

static bool readAdditionalFieldsValuesFromFile( std::ifstream& in, unsigned int nr_of_additional_fields, std::vector<double>& additional_fields_values )
{
  bool success = true;
  double field_value;

  for (int i = 0; i < nr_of_additional_fields && success; i++)
  {
    success = success && ( in>>field_value );
    if (success)
    {
      additional_fields_values.push_back(field_value);
    }
  }
  return success;
}

static bool readAdditionalFieldsNamesFromFile( std::ifstream& in, const unsigned int& nr_of_additional_fields, std::vector<std::string>& additional_fields_names)
{
  bool success = true;

  for( unsigned int i = 0; i < nr_of_additional_fields; i++ )
  {
    std::string field_name;
    success = success && (in >> field_name);
    if (success)
    {
      additional_fields_names.push_back(field_name);
    }
  }

  return success;
}

void ViewSpace::loadFromFile( std::string _filename )
{
  std::ifstream in(_filename, std::ifstream::in);

  unsigned int nr_of_views;
  bool success = (in >> nr_of_views);

  unsigned int nr_of_additional_fields;
  success = success && (in >> nr_of_additional_fields);

  std::vector<std::string> additional_fields_names;
  success = success && readAdditionalFieldsNamesFromFile(in, nr_of_additional_fields, additional_fields_names);

  if(!success)
    return;

  for( unsigned int i=0; i<nr_of_views; ++i )
  {
    View new_pose;
    success = readPoseFromFile(in, new_pose.pose());
    if(!success)
      return;

    new_pose.additionalFieldsNames() = additional_fields_names;
    success = readAdditionalFieldsValuesFromFile(in, nr_of_additional_fields, new_pose.additionalFieldsValues());
    if(!success)
      return;

    new_pose.bad() = false;
    new_pose.reachable() = true;
    new_pose.timesVisited() = 0;

    //view_space_.push_back(new_pose);
    views_index_map_[new_pose.index()] = new_pose;//view_space_.back();
  }
}

ViewSpace::Iterator ViewSpace::begin()
{
  return Iterator(views_index_map_.begin());
}

ViewSpace::ConstIterator ViewSpace::begin() const
{
  return ConstIterator(views_index_map_.begin());
}

ViewSpace::Iterator ViewSpace::end()
{
  return Iterator(views_index_map_.end());
}

ViewSpace::ConstIterator ViewSpace::end() const
{
  return ConstIterator(views_index_map_.end());
}

bool ViewSpace::empty() const
{
  return views_index_map_.empty();
}

void ViewSpace::recalculateIndexMap()
{
  /*views_index_map_.clear();
  
  for( View& view: view_space_ )
  {
    views_index_map_[view.index()] = view;
  }*/
}

// Iterator ***************************************************************************
  ViewSpace::Iterator::Iterator()
  {
    
  }
  
  ViewSpace::Iterator::Iterator(InternalIteratorType it)
  : it_(it)
  {
    
  }
  
  bool ViewSpace::Iterator::operator==(const Iterator& it) const
  {
    return it_==it.it_;
  }
  
  bool ViewSpace::Iterator::operator!=(const Iterator& it) const
  {
    return it_!=it.it_;
  }
  
  
  View& ViewSpace::Iterator::operator*() const
  {
    return it_->second;
  }
  
  View* ViewSpace::Iterator::operator->() const
  {
    return &it_->second;
  }
  
  
  ViewSpace::Iterator& ViewSpace::Iterator::operator++()
  {
    ++it_;
    return *this;
  }
  
  ViewSpace::Iterator ViewSpace::Iterator::operator++(int)
  {
    Iterator copy = *this;
    ++it_;
    return copy;
  }
  
  
  ViewSpace::Iterator& ViewSpace::Iterator::operator--()
  {
    --it_;
    return *this;
  }
  
  ViewSpace::Iterator ViewSpace::Iterator::operator--(int)
  {
    Iterator copy = *this;
    --it_;
    return copy;
  }
/* const iterator: */
  ViewSpace::ConstIterator::ConstIterator()
  {
    
  }
  
  ViewSpace::ConstIterator::ConstIterator(InternalIteratorType it)
  : it_(it)
  {
    
  }
  
  bool ViewSpace::ConstIterator::operator==(const ConstIterator& it) const
  {
    return it_==it.it_;
  }
  
  bool ViewSpace::ConstIterator::operator!=(const ConstIterator& it) const
  {
    return it_!=it.it_;
  }
  
  
  const View& ViewSpace::ConstIterator::operator*() const
  {
    return it_->second;
  }
  
  const View* ViewSpace::ConstIterator::operator->() const
  {
    return &it_->second;
  }
  
  
  ViewSpace::ConstIterator& ViewSpace::ConstIterator::operator++()
  {
    ++it_;
    return *this;
  }
  
  ViewSpace::ConstIterator ViewSpace::ConstIterator::operator++(int)
  {
    ConstIterator copy = *this;
    ++it_;
    return copy;
  }
  
  
  ViewSpace::ConstIterator& ViewSpace::ConstIterator::operator--()
  {
    --it_;
    return *this;
  }
  
  ViewSpace::ConstIterator ViewSpace::ConstIterator::operator--(int)
  {
    ConstIterator copy = *this;
    --it_;
    return copy;
  }

}

}
