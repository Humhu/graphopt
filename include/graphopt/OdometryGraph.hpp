#pragma once

#include "argus_utils/utils/MapUtils.hpp"
#include "graphopt/PoseGraph.hpp"
#include "graphopt/slam_traits.h"

#include <boost/foreach.hpp>
#include <iostream> // TODO

namespace argus
{

/*! \brief Provides an interface for managing a linked set of pose nodes. Allows
 * retrieval and insertion of nodes. */
template <class P, typename IndexType = boost::posix_time::ptime>
class OdometryGraph
: public PoseGraph <P, IndexType>
{
public:

	typedef typename PoseGraph<P,IndexType>::PoseType PoseType;
	typedef typename PoseGraph<P,IndexType>::NodeType NodeType;
	typedef typename PoseGraph<P,IndexType>::PriorType PriorType;
	typedef typename PoseGraph<P,IndexType>::EdgeType EdgeType;
	typedef typename PoseGraph<P,IndexType>::NoiseType NoiseType;
	typedef std::shared_ptr<OdometryGraph> Ptr;
	
	OdometryGraph( GraphOptimizer& s, bool optimize = true  )
	: PoseGraph<P,IndexType>( s, optimize ), _numPriors( 0 ) {}
	
	virtual IndexType EarliestIndex() const
	{
		return argus::get_lowest_key( _timeSeries );
	}

	virtual IndexType LatestIndex() const
	{
		return argus::get_highest_key( _timeSeries );
	}

	// If the time is within the graph time extents, it can be
	// automatically initialized
	virtual bool IsInitialized( const IndexType& ind ) const
	{
		return InRange( ind );
	}

	// An odometry graph is grounded so long as it has at least one prior
	virtual bool IsGrounded( const IndexType& ind ) const
	{
		return _numPriors > 0;
	}

	// Return whether a time is within the graph time extents
	bool InRange( const IndexType& ind ) const
	{
		if( _timeSeries.size() == 0 ) { return false; }
		return ind >= EarliestIndex() && ind <= LatestIndex();
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
		datum.node = std::make_shared <NodeType>();
		datum.node->init( pose );
		datum.toPrev = nullptr;
		
		_timeSeries[ ind ] = datum;
		this->AddGraphNode( datum.node );
		return datum.node;
	}

	/*! \brief Retrieve a node at the specified index if it exists. Creates the node
	 * if it is between the first and last index otherwise. Returns the node. */
	typename NodeType::Ptr RetrieveNode( const IndexType& ind )
	{
		if( _timeSeries.count( ind ) > 0 ) { return _timeSeries[ind].node; }
		return SplitOdometry( ind );
	}

	/*! \brief Removes the pose at the index and all connected factors from the graph.
	 * Note that this includes factors added from outside the pose graph. Fills the 
	 * resulting hole with a composed odometry factor. */
	virtual void RemoveNode( const IndexType& ind )
	{
		if( _timeSeries.count( ind ) == 0 ) { return; }

		const Datum& datum = _timeSeries[ ind ];
		_numPriors = _numPriors - datum.priors.size();
		this->RemoveGraphNode( datum.node );
		// NOTE Don't have to remove prior since remove_node will do it automatically?

		typename TimeSeries::iterator prevIter;
		bool hasLower = argus::get_closest_lesser( _timeSeries, ind, prevIter );
		Datum& prev = prevIter->second;

		typename TimeSeries::iterator nextIter;
		bool hasUpper = argus::get_closest_greater( _timeSeries, ind, nextIter );
		Datum& next = nextIter->second;

		// Fill in hole in graph by joining odometry
		if( hasUpper && hasLower )
		{
			PoseType sumOdom;
			typename isam::Slam_Traits<P>::CovarianceType sumCov;
			isam::Slam_Traits<P>::ComposeDisplacement
			    ( datum.toPrev->measurement(), next.toPrev->measurement(),
			      datum.toPrev->noise().cov(), next.toPrev->noise().cov(),
			      sumOdom, sumCov );
			next.toPrev = std::make_shared<EdgeType>
			    ( prev.node.get(), next.node.get(), sumOdom, isam::Covariance( sumCov ) );
				this->AddGraphEdge( next.toPrev );
		}
		// Else just clear odometry factor pointing to node we removed
		else if( hasUpper )
		{
			next.toPrev = nullptr;
		}

		// Don't erase until we're done using datum
		_timeSeries.erase( ind );
	}
	
	virtual void ClearNodes()
	{
		BOOST_FOREACH( typename TimeSeries::value_type& iter, _timeSeries )
		{
			this->RemoveGraphNode( iter.second.node );
		}
		_timeSeries.clear();
		_numPriors = 0;
	}

	/*! \brief Creates a node and adds an edge to the previous node. Index must
	 * come after the latest index. */
	virtual void CreateEdge( const IndexType& from, const IndexType& to,
	                         const PoseType& displacement, const NoiseType& noise )
	{
		typename NodeType::Ptr firstNode = RetrieveNode( from );
		typename NodeType::Ptr lastNode = RetrieveNode( to );
		if( !firstNode || !lastNode ) { return; }

		Datum& lastDatum = _timeSeries.at( to );
		if( lastDatum.toPrev )
		{
			this->RemoveGraphEdge( lastDatum.toPrev );
			lastDatum.toPrev.reset();
		}

		lastDatum.toPrev = std::make_shared <EdgeType>
		    ( firstNode.get(), lastNode.get(), displacement, noise );
		
			this->AddGraphEdge( lastDatum.toPrev );
	}
	
	void CreatePrior( const IndexType& ind, const PoseType& pose,
	                  const isam::Noise& noise )
	{
		if( !RetrieveNode( ind ) ) { return; }
		Datum& datum = _timeSeries[ ind ];
		typename PriorType::Ptr prior = std::make_shared <PriorType>
		    ( datum.node.get(), pose, noise );
		datum.priors.push_back( prior );
		this->AddGraphPrior( prior );
		++_numPriors;
	}

	/*! \brief Removes nodes that have only odometry factors. */
	void CompressGraph()
	{
		std::vector<IndexType> toRemove;
		BOOST_FOREACH( typename TimeSeries::value_type& iter, _timeSeries )
		{
			const IndexType& ind = iter.first;
			const Datum& data = iter.second;

			// Don't remove if there are any priors
			if( data.priors.count() > 0 ) { continue; }

			// Else collect all the local factors and check against the node's list
			std::set<isam::Factor*> localFactors;

			if( data.toPrev ) { localFactors.insert( data.toPrev.get() ); }

			typename TimeSeries::iterator nextIter;
			if( argus::get_closest_greater( _timeSeries, ind, nextIter ) )
			{
				localFactors.insert( nextIter->toPrev.get() );
			}

			const std::list<isam::Factor*>& factors = data.node->factors();
			bool remove = true;
			BOOST_FOREACH( isam::Factor* factor, factors )
			{
				// If the node has a non-local factor, we can't remove it
				if( localFactors.count( factor ) == 0 )
				{
					remove = false;
					break;
				}
			}

			if( remove ) { toRemove.push_back( ind ); }
		}

		BOOST_FOREACH( const IndexType& ind, toRemove )
		{
			RemoveNode( ind );
		}
	}

private:
	
	// Inserts a node at the specified index in between two existing nodes.
	typename NodeType::Ptr SplitOdometry( const IndexType& ind )
	{
		typename TimeSeries::iterator prevIter, nextIter;
		// Make sure index has previous and following items
		if( !argus::get_closest_lesser( _timeSeries, ind, prevIter ) || 
		    !argus::get_closest_greater( _timeSeries, ind, nextIter ) )
		{
			return nullptr;
		}
		
		IndexType prevTime = prevIter->first;
		IndexType nextTime = nextIter->first;
		
		double prevDt = IndexTraits<IndexType>::Difference(ind, prevTime);
		double nextDt = IndexTraits<IndexType>::Difference(nextTime, ind);
		double prevProp = prevDt / ( prevDt + nextDt );
		double nextProp = 1.0 - prevProp;
		
		// Split odometry
		typename EdgeType::Ptr odometry = nextIter->second.toPrev;
		if( !odometry ) 
		{ 
			// NOTE This should really not happen...
			std::cerr << "NextIter doesn't have odometry!" << std::endl;
			return nullptr; 
		}

		// Add newest timepoint
		Datum midDatum;
		midDatum.node = std::make_shared<NodeType>();
		midDatum.toPrev = std::make_shared <EdgeType>
		    ( prevIter->second.node.get(), midDatum.node.get(),
		      isam::Slam_Traits<P>::ScaleDisplacement( odometry->measurement(), prevProp ), 
		      isam::SqrtInformation( prevProp * odometry->sqrtinf() ) );
		
		_timeSeries[ ind ] = midDatum;
		
		// Update last timepoint with split odometry
		nextIter->second.toPrev = std::make_shared <EdgeType>
		    ( midDatum.node.get(), nextIter->second.node.get(), 
		      isam::Slam_Traits<P>::ScaleDisplacement( odometry->measurement(), nextProp ),
		      isam::SqrtInformation( nextProp * odometry->sqrtinf() ) );

		this->RemoveGraphEdge( odometry );
		this->AddGraphNode( midDatum.node );
		this->AddGraphEdge( midDatum.toPrev );
		this->AddGraphEdge( nextIter->second.toPrev );
		return midDatum.node;
	}
	
	struct Datum
	{
		typename NodeType::Ptr node;
		typename EdgeType::Ptr toPrev;
		std::vector<typename PriorType::Ptr> priors;
	};
	
	typedef std::map <IndexType, Datum> TimeSeries;
	TimeSeries _timeSeries;	
	unsigned int _numPriors;
};

}
