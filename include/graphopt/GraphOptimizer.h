# pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <isam/Slam.h>
#include <memory>

namespace argus
{

/*! \brief Wraps the ISAM graph optimizer with ROS functionality.
*/
class GraphOptimizer
{
public:

	GraphOptimizer();

	/*! \brief Construct an optimizer from a private namespace. */
	GraphOptimizer( ros::NodeHandle& ph );

	template <typename N>
	void AddNode( const std::shared_ptr<N>& node )
	{
		ROS_INFO_STREAM( "Adding node to graph: " << node.get() );
		_optimizer->add_node( node.get() );
	}

	template <typename F>
	void AddFactor( const std::shared_ptr<F>& factor )
	{
		ROS_INFO_STREAM( "Adding factor to graph: " << factor.get() );		
		_optimizer->add_factor( factor.get() );
	}

	template <typename N>
	void RemoveNode( const std::shared_ptr<N>& node )
	{
		ROS_INFO_STREAM( "Removing node from graph: " << node.get() );
		_optimizer->remove_node( node.get() );
	}

	template <typename F>
	void RemoveFactor( const std::shared_ptr<F>& factor )
	{
		ROS_INFO_STREAM( "Removing factor from graph: " << factor.get() );
		_optimizer->remove_factor( factor.get() );
	}

	/*! \brief Retrieve a reference to the raw optimizer object. */
	isam::Slam& GetOptimizer();
	const isam::Slam& GetOptimizer() const;

private:

	isam::Slam::Ptr _optimizer;
};

}