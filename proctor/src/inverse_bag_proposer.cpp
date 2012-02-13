#include "proctor/inverse_bag_proposer.h"

namespace pcl
{
  
  namespace proctor
  {
    double
    InverseBagProposer::getVotes(Entry &query, Entry &match)
    {
      int max_votes = 1;

      double votes = 0;
      for (unsigned int pi = 0; pi < query.features->size(); pi++) {
        vector<int> indices;
        vector<float> distances;

        int num_found = match.tree->nearestKSearch(*query.features, pi, max_votes, indices, distances);

        for (int ri = 0; ri < num_found; ri++) {
          votes += 1.0 / distances[ri];
        }
      }

      return votes;
    }

  }

}
