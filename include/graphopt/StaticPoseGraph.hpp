#pragma once

#include "graphopt/PoseGraph.hpp"

namespace argus
{

// A pose graph interface around a static pose instance
template <class P, typename IndexType = boost::posix_time::ptime>
class StaticPoseGraph
: public PoseGraph<P,IndexType>
{
public:

	typedef typename PoseGraph<P,IndexType>::PoseType PoseType;
	typedef typename PoseGraph<P,IndexType>::NodeType NodeType;
	typedef typename PoseGraph<P,IndexType>::PriorType PriorType;
	typedef typename PoseGraph<P,IndexType>::EdgeType EdgeType;
	typedef typename PoseGraph<P,IndexType>::NoiseType NoiseType;
	typedef std::shared_ptr<StaticPoseGraph> Ptr;

	StaticPoseGraph( GraphOptimizer& s ) 
	: PoseGraph<P,IndexType>( s ) {}

	virtual IndexType EarliestIndex() const
	{
		return IndexTraits<IndexType>::Earliest();
	}

	virtual IndexType LatestIndex() const
	{
		return IndexTraits<IndexType>::Latest();
	}

	virtual bool IsGrounded( const IndexType& ind ) const 
	{
		return _priors.size() > 0;
	}

	virtual typename NodeType::Ptr CreateNode( const IndexType& ind,
	                                           const PoseType& pose )
	{
		if( _node ) 
		{ 
			_node->init( pose );
			return _node; 
		}

		_node = std::make_shared<NodeType>();
		PoseGraph<P,IndexType>::_graph.AddNode( _node );
		_node->init( pose );
		return _node;
	}

	virtual typename NodeType::Ptr RetrieveNode( const IndexType& ind )
	{
		return _node;
	}

	virtual void RemoveNode( const IndexType& ind )
	{
		if( !_node ) { return; }
		PoseGraph<P,IndexType>::_graph.RemoveNode( _node );
		_node.reset();
		_priors.clear();
	}

	virtual void ClearNodes()
	{
		RemoveNode( IndexType() );
	}

	virtual void CreatePrior( const IndexType& ind, const PoseType& pose,
	                                   const isam::Noise& noise )
	{
		typename NodeType::Ptr n = RetrieveNode( ind );
		
		typename PriorType::Ptr prior = std::make_shared <PriorType>
		    ( _node.get(), pose, noise );
		PoseGraph<P,IndexType>::_graph.AddFactor( prior );
		_priors.push_back( prior );
	}

	virtual void CreateEdge( const IndexType& from, const IndexType& to,
	                         const PoseType& pose, const NoiseType& noise ) {}

private:

	typename NodeType::Ptr _node;
	std::vector<typename PriorType::Ptr> _priors;

};

}