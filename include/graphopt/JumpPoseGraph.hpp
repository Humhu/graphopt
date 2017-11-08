#pragma once

#include "graphopt/PoseGraph.hpp"

namespace argus
{
template <class P, typename IndexType = boost::posix_time::ptime>
class JumpPoseGraph
	: public PoseGraph<P, IndexType>
{
public:

	typedef typename PoseGraph<P, IndexType>::PoseType PoseType;
	typedef typename PoseGraph<P, IndexType>::NodeType NodeType;
	typedef typename PoseGraph<P, IndexType>::PriorType PriorType;
	typedef typename PoseGraph<P, IndexType>::EdgeType EdgeType;
	typedef typename PoseGraph<P, IndexType>::NoiseType NoiseType;
	typedef std::shared_ptr<JumpPoseGraph> Ptr;

	JumpPoseGraph( GraphOptimizer& s, bool optimize )
		: PoseGraph<P, IndexType>( s, optimize ) {}

	virtual IndexType EarliestIndex() const
	{
		return argus::get_lowest_key( _data );
	}

	virtual IndexType LatestIndex() const
	{
		return argus::get_highest_key( _data );
	}

	// If a node exists in the graph, it must be initialized
	virtual bool IsInitialized( const IndexType& ind ) const
	{
		return _data.count( ind ) != 0;
	}

	// Since there are no strong edge connections, just see if there are priors
	virtual bool IsGrounded( const IndexType& ind ) const
	{
		if( _data.count( ind ) == 0 ) { return false; }
		return _data.at( ind ).priors.size() > 0;
	}

	virtual typename NodeType::Ptr CreateNode( const IndexType& ind,
	                                           const PoseType& pose )
	{
		typename NodeType::Ptr node = RetrieveNode( ind );
		if( node )
		{
			node->init( pose );
			return node;
		}

		Datum datum;
		datum.node = std::make_shared<NodeType>();
		datum.node->init( pose );
		_data[ind] = datum;
		this->AddGraphNode( datum.node );
		return datum.node;
	}

	virtual typename NodeType::Ptr RetrieveNode( const IndexType& ind )
	{
		if( _data.count( ind ) == 0 ) { return nullptr; }
		return _data.at( ind ).node;
	}

	virtual void RemoveNode( const IndexType& ind )
	{
		if( _data.count( ind ) > 0 )
		{
			this->RemoveGraphNode( _data[ind].node );
			_data.erase( ind );
		}
	}

	virtual void ClearNodes()
	{
		BOOST_FOREACH( const typename DataMap::value_type & iter, _data )
		{
			RemoveNode( iter.first );
		}
		_data.clear();
	}

	virtual void CreatePrior( const IndexType& ind, const PoseType& pose,
	                          const isam::Noise& noise )
	{
		if( _data.count( ind ) == 0 ) { return; }

		Datum& d = _data.at( ind );
		typename PriorType::Ptr prior = std::make_shared<PriorType>
		                                    ( d.node.get(), pose, noise );
		this->AddGraphPrior( prior );
		d.priors.push_back( prior );
	}

	virtual void CreateEdge( const IndexType& from, const IndexType& to,
	                         const PoseType& pose, const NoiseType& noise )
	{
		// NOTE Does nothing
		// TODO Notify the user via std::cerr?
	}

private:

	struct Datum
	{
		typename NodeType::Ptr node;
		std::vector<typename PriorType::Ptr> priors;
	};

	typedef std::map<IndexType, Datum> DataMap;
	DataMap _data;
};
}